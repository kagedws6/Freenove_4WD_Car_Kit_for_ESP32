"""Read-only TCP bench checks: status, malformed-command rejection and reconnect.

Does not send a valid movement request. Motor battery should remain off for initial
bring-up. Status reports commanded PWM, not physical motor movement.
"""
import argparse
import socket
import time

def connect(host):
    s = socket.create_connection((host, 4000), timeout=3)
    s.settimeout(2)
    return s, s.makefile('rb')

def status(s, stream):
    s.sendall(b'CMD_STATUS\n')
    while True:
        line = stream.readline(512).decode('ascii').strip()
        if not line:
            raise AssertionError('Connection closed')
        if line.startswith('ERR#'):
            continue
        fields = line.split('#')
        assert len(fields) == 9 and fields[:2] == ['CMD_STATUS', '1'], line
        assert fields[2:4] == ['0', '0'] and fields[4] == '2200', line
        assert fields[5:7] == ['0', '0'], line
        return fields

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host')
    args = parser.parse_args()
    for attempt in range(2):
        s, stream = connect(args.host)
        try:
            status(s, stream)
            for invalid in (b'CMD_MOTOR#9999999999999#0#0#0\n',
                            b'CMD_DRIVE#no#0#10\n', b'UNKNOWN\n',
                            b'X' * 150 + b'\n', b'CMD_STOP#1\n'):
                s.sendall(invalid)
                assert stream.readline(512).startswith(b'ERR#'), invalid
                status(s, stream)
            s.sendall(b'CMD_MOTOR#')
            time.sleep(.35)
            assert stream.readline(512).startswith(b'ERR#')
            s.sendall(b'0#0#0#0\n') # Expired frame tail must not become a command.
            assert stream.readline(512).startswith(b'ERR#')
            status(s, stream)
            s.sendall(b'CMD_STOP\n')
            result = status(s, stream)
            print(f'PASS connection {attempt+1}: idle, zero PWM, bad frames rejected; camera_ready={result[8]}')
        finally:
            stream.close()
            s.close()
        time.sleep(.1)
    print('PASS: command-client reconnect')

if __name__ == '__main__':
    main()
