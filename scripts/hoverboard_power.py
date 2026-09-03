#!/usr/bin/env python3

import argparse
import os
import time


DEFAULT_STATE_FILE = '/tmp/boxbot_hoverboard_power_on'


def _bool_arg(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _read_sense(GPIO, sense_pin, active_high):
    raw = bool(GPIO.input(sense_pin))
    return raw if active_high else not raw, raw


def _pulse_switch(GPIO, switch_pin, pulse_seconds):
    GPIO.output(switch_pin, GPIO.HIGH)
    time.sleep(pulse_seconds)
    GPIO.output(switch_pin, GPIO.LOW)


def _write_state(path):
    with open(path, 'w', encoding='utf-8') as state_file:
        state_file.write(str(time.time()))


def _clear_state(path):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass


def main():
    parser = argparse.ArgumentParser(
        description='Latch hoverboard power with a GPIO momentary switch.'
    )
    parser.add_argument(
        'action',
        choices=('on', 'off', 'pulse', 'status'),
        help='Power action to perform.',
    )
    parser.add_argument('--sense-pin', type=int, default=7)
    parser.add_argument('--switch-pin', type=int, default=11)
    parser.add_argument('--pulse-seconds', type=float, default=0.2)
    parser.add_argument('--settle-seconds', type=float, default=0.5)
    parser.add_argument('--sense-active-high', type=_bool_arg, default=True)
    parser.add_argument('--once-per-session', type=_bool_arg, default=True)
    parser.add_argument('--state-file', default=DEFAULT_STATE_FILE)
    parser.add_argument(
        '--force',
        action='store_true',
        help='Pulse even when sense/session state says it is unnecessary.',
    )
    args = parser.parse_args()

    import Jetson.GPIO as GPIO

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(args.switch_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(args.sense_pin, GPIO.IN)

    try:
        is_on, raw_sense = _read_sense(GPIO, args.sense_pin, args.sense_active_high)
        state_exists = os.path.exists(args.state_file)

        print(
            f'Hoverboard sense raw={int(raw_sense)} '
            f'interpreted={"on" if is_on else "off"} '
            f'state_file={"present" if state_exists else "absent"}'
        )

        if args.action == 'status':
            return 0

        if args.action == 'pulse':
            print('Pulsing hoverboard power switch...')
            _pulse_switch(GPIO, args.switch_pin, args.pulse_seconds)
            return 0

        if args.action == 'on':
            if is_on and not args.force:
                print('Hoverboard already appears to be on.')
                _write_state(args.state_file)
                return 0

            if args.once_per_session and state_exists and not args.force:
                print(
                    'Startup pulse was already sent this session; '
                    'not toggling hoverboard again.'
                )
                return 0

            print('Turning on hoverboard...')
            _pulse_switch(GPIO, args.switch_pin, args.pulse_seconds)
            time.sleep(args.settle_seconds)

            is_on_after, raw_after = _read_sense(
                GPIO, args.sense_pin, args.sense_active_high
            )
            print(
                f'Hoverboard sense after pulse raw={int(raw_after)} '
                f'interpreted={"on" if is_on_after else "off"}'
            )
            _write_state(args.state_file)
            return 0

        if args.action == 'off':
            if not is_on and not args.force:
                print('Hoverboard already appears to be off.')
                _clear_state(args.state_file)
                return 0

            print('Turning off hoverboard...')
            _pulse_switch(GPIO, args.switch_pin, args.pulse_seconds)
            _clear_state(args.state_file)
            return 0

    finally:
        GPIO.cleanup()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
