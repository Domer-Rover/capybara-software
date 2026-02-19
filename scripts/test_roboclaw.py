from basicmicro import Basicmicro

PORT = "/dev/ttyTHS1"

# RoboClaw supports these baud rates
BAUD_RATES = [38400]

# Our three RoboClaws: top=128, middle=129, bottom=130
ADDRESSES = [0x80, 0x81, 0x82]  # 128, 129, 130

def test_address(controller, address):
    results = {}

    try:
        main_batt = controller.ReadMainBatteryVoltage(address)
        results['main_batt'] = f"{main_batt[1]/10.0}V" if main_batt[0] else "read failed"
    except Exception as e:
        results['main_batt'] = f"ERROR: {e}"

    try:
        logic_batt = controller.ReadLogicBatteryVoltage(address)
        results['logic_batt'] = f"{logic_batt[1]/10.0}V" if logic_batt[0] else "read failed"
    except Exception as e:
        results['logic_batt'] = f"ERROR: {e}"

    try:
        temp = controller.ReadTemp(address)
        results['temp'] = f"{temp[1]/10.0}°C" if temp[0] else "read failed"
    except Exception as e:
        results['temp'] = f"ERROR: {e}"

    return results


for baud in BAUD_RATES:
    print(f"\n--- Trying {PORT} at {baud} baud ---")
    try:
        with Basicmicro(PORT, baud) as controller:
            any_responded = False
            for addr in ADDRESSES:
                print(f"  Address 0x{addr:02X} ({addr}):", end=" ", flush=True)
                try:
                    main_batt = controller.ReadMainBatteryVoltage(addr)
                    if main_batt[0]:
                        print(f"RESPONDING! Main batt: {main_batt[1]/10.0}V")
                        results = test_address(controller, addr)
                        for k, v in results.items():
                            print(f"    {k}: {v}")
                        any_responded = True
                    else:
                        print("no response (read returned False)")
                except Exception as e:
                    print(f"timeout/error")

            if any_responded:
                print(f"\n  *** Found RoboClaws at {baud} baud! ***")
    except Exception as e:
        print(f"  Failed to open port: {e}")
