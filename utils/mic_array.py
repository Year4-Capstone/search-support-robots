import sys, time, argparse, queue, json, threading
import numpy as np
import sounddevice as sd

TARGET_SR = 16000
DEVICE_MATCH_KEYS = ["respeaker", "usb mic array", "seeed", "xmos"]
PREFERRED_CHANNELS = [6, 8, 4, 2, 1]
BEAMFORMED_DEFAULT_INDEX = 5

def pick_respeaker_device():
    devices = sd.query_devices()
    hostapis = sd.query_hostapis()
    def hostapi_name(idx):
        try: return hostapis[devices[idx]['hostapi']]['name']
        except Exception: return "?"
    candidates = []
    for i, d in enumerate(devices):
        name = (d.get('name') or '').lower()
        if d.get('max_input_channels', 0) > 0 and any(k in name for k in DEVICE_MATCH_KEYS):
            candidates.append((i, d))
    if not candidates:
        di = sd.default.device[0]
        dn = devices[di]['name']
        print(f"Couldn’t match a ReSpeaker by name. Using default input: '{dn}' ({hostapi_name(di)})")
        return di
    candidates.sort(key=lambda t: t[1].get('max_input_channels', 0), reverse=True)
    i, d = candidates[0]
    print(f"Using input: '{d['name']}' via {hostapi_name(i)} ({d.get('max_input_channels', 0)} ch max)")
    return i

def choose_channel_count(device_index):
    max_ch = sd.query_devices(device_index)['max_input_channels']
    for ch in PREFERRED_CHANNELS:
        if ch <= max_ch: return ch
    return max(1, max_ch)

def make_resampler(in_sr, out_sr):
    if in_sr == out_sr: return lambda x: x
    ratio = out_sr / in_sr
    def resample(x):
        x = np.asarray(x)
        if x.ndim == 1:
            n_in = x.shape[0]; n_out = int(np.round(n_in * ratio))
            t_old = np.linspace(0.0, 1.0, num=n_in, endpoint=False)
            t_new = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
            return np.interp(t_new, t_old, x).astype(np.float32)
        n_in = x.shape[0]; n_out = int(np.round(n_in * ratio))
        t_old = np.linspace(0.0, 1.0, num=n_in, endpoint=False)
        t_new = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
        out = np.empty((n_out, x.shape[1]), dtype=np.float32)
        for c in range(x.shape[1]):
            out[:, c] = np.interp(t_new, t_old, x[:, c])
        return out
    return resample

def main():
    ap = argparse.ArgumentParser(description="ReSpeaker live STT + DOA (Vosk)")
    ap.add_argument("--model", required=True, help="Path to an unzipped Vosk model dir")
    ap.add_argument("--rate", type=int, default=TARGET_SR, help="Capture sample rate (resampled to 16 kHz for Vosk)")
    ap.add_argument("--device", default=None, help="Device index or name snippet")
    ap.add_argument("--channel", type=int, default=None, help="0-based channel for STT input")
    ap.add_argument("--no-led", action="store_true", help="Disable LED ring even if available")
    ap.add_argument("--show-partials", action="store_true", help="Print partial words while speaking")
    ap.add_argument("--show-doa", action="store_true", help="Poll and print Direction of Arrival (0–359°)")
    ap.add_argument("--doa-interval", type=float, default=0.4, help="Seconds between DOA polls")
    args = ap.parse_args()

    pixel = None
    if not args.no_led:
        try:
            from pixel_ring import usb_pixel_ring_v2, usb_pixel_ring
            try: pixel = usb_pixel_ring_v2.UsbPixelRingV2()
            except Exception: pixel = usb_pixel_ring.UsbPixelRing()
            try:
                pixel.set_brightness(16)
                pixel.wakeup()
            except Exception:
                pixel = None
        except Exception:
            pixel = None

    if args.device is None:
        device_index = pick_respeaker_device()
    else:
        try:
            device_index = int(args.device)
        except ValueError:
            device_index = None
            for i, d in enumerate(sd.query_devices()):
                if args.device.lower() in (d.get('name') or '').lower():
                    device_index = i; break
            if device_index is None:
                print(f"Couldn’t find device containing '{args.device}'."); sys.exit(1)

    channels = choose_channel_count(device_index)
    in_sr = args.rate
    ch_idx = (int(args.channel) if args.channel is not None
              else (BEAMFORMED_DEFAULT_INDEX if channels >= 6 else 0))
    if ch_idx < 0 or ch_idx >= channels:
        print(f"Channel index {ch_idx} out of range for {channels} channels."); sys.exit(1)

    calib_secs = 1.5
    tmp = []
    def calib_cb(indata, frames, time_info, status):
        if status: print(f"[calib status] {status}")
        tmp.append(indata.copy())
    print("Calibrating levels for", calib_secs, "s...")
    with sd.InputStream(device=device_index, channels=channels, samplerate=in_sr, dtype="float32",
                        blocksize=int(in_sr // 10), callback=calib_cb):
        sd.sleep(int(calib_secs * 1000))
    calib = np.concatenate(tmp, axis=0) if tmp else np.zeros((int(in_sr * calib_secs), channels), np.float32)
    rms = np.sqrt((calib ** 2).mean(axis=0) + 1e-12)
    best = int(np.argmax(rms))
    print("sPer-channel RMS:", ["{:.3f}".format(x) for x in rms])
    if args.channel is None:
        ch_idx = best
        print(f"Auto-selecting channel #{ch_idx}")

    try:
        from vosk import Model, KaldiRecognizer
    except Exception:
        print("Vosk not installed. Run:  pip install vosk"); sys.exit(1)
    try:
        model = Model(args.model)
    except Exception as e:
        print(f"Failed to load Vosk model at '{args.model}': {e}"); sys.exit(1)
    recognizer = KaldiRecognizer(model, TARGET_SR)
    recognizer.SetWords(True)

    current_doa = {"angle": None}
    doa_lock = threading.Lock()
    stop_doa = threading.Event()
    doa_thread = None

    def get_doa_str():
        with doa_lock:
            ang = current_doa["angle"]
        return f"{ang:03d}°" if isinstance(ang, int) else "NA°"

    if args.show_doa:
        tuner = None
        try:
            import usb.core
            dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if dev is None:
                raise RuntimeError("ReSpeaker HID device not found via PyUSB")
            try:
                from usb_4_mic_array.tuning import Tuning
            except Exception:
                from tuning import Tuning
            tuner = Tuning(dev)
        except Exception as e:
            print(" DOA unavailable:", e)
            print(" Ensure: WinUSB via Zadig on the HID interface, pyusb installed, and libusb-1.0.dll on PATH.")

        if tuner is not None:
            def poll_doa():
                last = None
                while not stop_doa.is_set():
                    try:
                        angle = int(tuner.direction)
                        with doa_lock:
                            current_doa["angle"] = angle
                        if pixel is not None and angle != last:
                            try: pixel.set_direction(angle)
                            except Exception: pass
                        last = angle
                    except Exception:
                        pass
                    time.sleep(max(0.05, args.doa_interval))
            doa_thread = threading.Thread(target=poll_doa, daemon=True)
            doa_thread.start()
            print("DOA polling: enabled")

    q = queue.Queue()
    resample = make_resampler(in_sr, TARGET_SR)

    def callback(indata, frames, time_info, status):
        if status: print(f"\n[stream status] {status}", file=sys.stderr)
        x = indata.astype(np.float32, copy=False)
        mono = x[:, ch_idx] if x.ndim == 2 else x
        mono_rs = resample(mono)
        pcm16 = (np.clip(mono_rs, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
        q.put(pcm16)

    print(f"Opening input stream on device={device_index}, channels={channels}, rate={in_sr} Hz")
    print(f"Using Vosk model at: {args.model}")
    print(f"Feeding channel #{ch_idx}")
    print("Speak!  (Ctrl+C to stop)\n")

    try:
        with sd.InputStream(device=device_index, channels=channels, samplerate=in_sr,
                            dtype="float32", blocksize=int(in_sr // 10), callback=callback):
            if pixel is not None:
                try: pixel.wakeup()
                except Exception: pass
            buffer_concat = b""
            bite_bytes = int(TARGET_SR * 0.25) * 2
            while True:
                chunk = q.get()
                buffer_concat += chunk
                if len(buffer_concat) >= bite_bytes:
                    if recognizer.AcceptWaveform(buffer_concat):
                        res = json.loads(recognizer.Result())
                        text = res.get("text", "").strip()
                        if text:
                            print(f"[{get_doa_str()}] {text}")
                    else:
                        # partials
                        pres = json.loads(recognizer.PartialResult())
                        part = pres.get("partial", "").strip()
                        if part and args.show_partials:
                            print(f"[{get_doa_str()}] {part}", end="\r", flush=True)
                    buffer_concat = b""
    except KeyboardInterrupt:
        pass
    finally:
        if pixel is not None:
            try: pixel.think(); time.sleep(0.5); pixel.off()
            except Exception: pass
        if doa_thread is not None:
            stop_doa.set(); doa_thread.join(timeout=1.0)

    try:
        final = json.loads(recognizer.FinalResult())
        text = final.get("text", "").strip()
        if text:
            print(f"\n[{get_doa_str()}] {text}")
    except Exception:
        pass
    print("\nStopped.")

if __name__ == "__main__":
    main()
