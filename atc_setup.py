#!/usr/bin/env python
"""Configure Feetech motor IDs for an Automatic Tool Changer (ATC)."""

import argparse

import scservo_sdk as scs

# SC-series motors speak protocol 1; ST/SMS-series (sts3215, ...) speak protocol 0.
SCS_MODELS = {"scs0009"}
SCRIPT_VERSION = "2026-06-07-direct-feetech-sdk"
BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]
IDS = range(1, 21)

# Feetech EEPROM registers used by both SC and ST/SMS families.
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_LOCK = 55
BAUD_CODE_1M = 0


class EchoFreePortHandler(scs.PortHandler):
    """Discard TX echo from the Waveshare half-duplex single-wire bus."""

    def openPort(self):
        result = super().openPort()
        if result and hasattr(self, "ser"):
            self.ser.timeout = 0.02
        return result

    def writePort(self, packet):
        result = super().writePort(packet)
        if hasattr(self, "ser"):
            self.ser.read(len(packet))
        return result


def protocol_for(model):
    return 1 if model in SCS_MODELS else 0


def comm_success():
    return getattr(scs, "COMM_SUCCESS", 0)


def require_ok(packet_handler, result, error, action):
    if result != comm_success():
        raise RuntimeError(f"{action}: {packet_handler.getTxRxResult(result)}")
    if error:
        print(f"  Warning: {action}: {packet_handler.getRxPacketError(error)}")


def open_bus(port, baudrate, protocol_version):
    port_handler = EchoFreePortHandler(port)
    packet_handler = scs.PacketHandler(protocol_version)
    if not port_handler.openPort():
        raise RuntimeError(f"Cannot open port {port}")
    if not port_handler.setBaudRate(baudrate):
        port_handler.closePort()
        raise RuntimeError(f"Cannot set baudrate {baudrate} on {port}")
    return port_handler, packet_handler


def ping_id(packet_handler, port_handler, motor_id):
    model_number, result, error = packet_handler.ping(port_handler, motor_id)
    if result != comm_success():
        return None
    return model_number, error


def find_one_motor(port, protocol_version):
    found = []
    for baudrate in BAUDRATES:
        port_handler, packet_handler = open_bus(port, baudrate, protocol_version)
        try:
            for motor_id in IDS:
                hit = ping_id(packet_handler, port_handler, motor_id)
                if hit is None:
                    continue
                model_number, error = hit
                err_msg = packet_handler.getRxPacketError(error) if error else "OK"
                print(f"  Found baud={baudrate} id={motor_id} model={model_number} status={err_msg}")
                found.append((baudrate, motor_id, model_number, error))
        finally:
            port_handler.closePort()

    if not found:
        raise RuntimeError("No motor found on IDs 1-20 at known Feetech baudrates")
    if len(found) > 1:
        details = ", ".join(f"baud={baud} id={motor_id}" for baud, motor_id, _, _ in found)
        raise RuntimeError(f"More than one motor responded ({details}). Connect exactly one motor.")
    return found[0]


def write_setup(port, baudrate, protocol_version, current_id, target_id):
    port_handler, packet_handler = open_bus(port, baudrate, protocol_version)
    try:
        print("  Unlocking EEPROM...")
        result, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_LOCK, 0)
        require_ok(packet_handler, result, error, "unlock EEPROM")

        if current_id != target_id:
            print(f"  Writing ID {current_id} -> {target_id}...")
            result, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_ID, target_id)
            require_ok(packet_handler, result, error, "write ID")
            current_id = target_id
        else:
            print(f"  ID already {target_id}")

        print("  Writing baudrate -> 1 Mbps...")
        result, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_BAUD_RATE, BAUD_CODE_1M)
        require_ok(packet_handler, result, error, "write baudrate")

        print("  Locking EEPROM...")
        result, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_LOCK, 1)
        require_ok(packet_handler, result, error, "lock EEPROM")
    finally:
        port_handler.closePort()


def verify_target(port, protocol_version, target_id):
    port_handler, packet_handler = open_bus(port, 1_000_000, protocol_version)
    try:
        hit = ping_id(packet_handler, port_handler, target_id)
        if hit is None:
            raise RuntimeError(f"Could not verify target ID {target_id} at 1 Mbps")
        model_number, error = hit
        status = packet_handler.getRxPacketError(error) if error else "OK"
        print(f"  Verified baud=1000000 id={target_id} model={model_number} status={status}")
    finally:
        port_handler.closePort()


def setup_motor(port, motor_id, label, model):
    print(f"\n{'=' * 60}")
    print(f"Setting up: {label} (target ID {motor_id}, model {model})")
    print(f"{'=' * 60}")
    input(f"Connect ONLY this motor to {port}, then press ENTER...")

    protocol_version = protocol_for(model)
    print("Scanning concrete IDs 1-20; not using broadcast ID 254...")
    baudrate, current_id, _, _ = find_one_motor(port, protocol_version)
    write_setup(port, baudrate, protocol_version, current_id, motor_id)
    verify_target(port, protocol_version, motor_id)
    print(f"  Done: ID={motor_id} set, baudrate programmed to 1 Mbps")


def main():
    parser = argparse.ArgumentParser(description="Configure ATC Feetech motor IDs")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM1")
    parser.add_argument(
        "--target",
        required=True,
        choices=["atc", "tool", "all"],
        help="atc = lock motor only; tool = tool motor(s); all = ATC + tool",
    )
    parser.add_argument(
        "--motors",
        type=int,
        choices=[1, 2],
        default=1,
        help="Number of tool motors (default: 1). Only relevant for --target tool or all.",
    )
    parser.add_argument("--model", default="sts3215",
                        help="Motor model for --target atc or tool (default: sts3215)")
    parser.add_argument("--atc-model", default="sts3215",
                        help="ATC motor model for --target all (default: sts3215)")
    parser.add_argument("--tool-model", default="scs0009",
                        help="Tool motor model for --target all (default: scs0009)")
    args = parser.parse_args()

    steps = []
    if args.target == "atc":
        steps.append((1, "ATC lock mechanism", args.model))
    elif args.target == "tool":
        steps.append((2, "Tool motor 1", args.model))
        if args.motors == 2:
            steps.append((3, "Tool motor 2", args.model))
    elif args.target == "all":
        steps.append((1, "ATC lock mechanism", args.atc_model))
        steps.append((2, "Tool motor 1", args.tool_model))
        if args.motors == 2:
            steps.append((3, "Tool motor 2", args.tool_model))

    print("ATC Motor Setup")
    print(f"Script: {SCRIPT_VERSION}")
    print(f"Port  : {args.port}")
    print(f"Steps : {len(steps)}")

    for motor_id, label, model in steps:
        setup_motor(args.port, motor_id, label, model)

    print(f"\n{'=' * 60}")
    print("Setup complete:")
    for motor_id, label, model in steps:
        print(f"  {label:25s} -> ID {motor_id}  ({model})")


if __name__ == "__main__":
    main()
