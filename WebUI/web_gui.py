from flask import Flask, render_template, jsonify, request, send_from_directory
import serial
import serial.tools.list_ports
import struct
import threading
import time
import os

app = Flask(__name__)

# USB CDC Configuration
# The device will enumerate as two CDC serial ports
# CDC_MAIN (port 0) - for waveform and logic analyzer commands
# CDC_LOGIC (port 1) - for SUMP logic analyzer protocol
VID = 0xCafe
PID = 0x4002  # Calculated from tusb_config.h: 0x4000 | _PID_MAP(CDC, 0)
CDC_MAIN_INTERFACE = 0  # First CDC port for commands

# Global device handle
ser = None
device_lock = threading.Lock()

# Command mappings
waveform_map = {
    "sine": 0,
    "square": 1,
    "triangle": 2,
    "ramp": 3,
    "pulse": 4
}

COMMAND_SET_WAVEFORM = 2
COMMAND_PING = 15
COMMAND_USB_SPEED_TEST = 16

la_commands = {
    "la_start": 3,
    "la_stop": 4,
    "la_get_chunk": 5,
    "la_config": 6
}

psu_commands = {
    "psu_on": 3,
    "psu_off": 4,
    "psu_query_alive": 11,   # COMMAND_PD_ALIVE
    "psu_query_capabs": 12,  # COMMAND_PD_QUERY_CAPABS
    "psu_query_status": 13, # COMMAND_PD_QUERY_STATUS
    "psu_config": 14,       # 0_0
}

# Current mapping for PPS/AVS PDOs (from AP33772S firmware)
PPS_CURRENT_MAP = {
    0: 1.24, 1: 1.49, 2: 1.74, 3: 1.99, 4: 2.24,
    5: 2.49, 6: 2.74, 7: 2.99, 8: 3.24, 9: 3.49,
    10: 3.74, 11: 3.99, 12: 4.24, 13: 4.49, 14: 4.74, 15: 5.0
}

# Packet format constants
PACKET_START_MARKER_REQUEST = 0xAA
PACKET_START_MARKER_RESPONSE = 0x55
PACKET_HEADER_SIZE = 3

# Error code mappings from snap_master.h and packet.h
error_codes = {
    0: "SNAP_OK - Success",
    1: "SNAP_ERR_NULL_PARAM - NULL parameter (hi2c/cmd/resp/buffer)",
    2: "SNAP_ERR_I2C_TX - I2C transmit failed",
    3: "SNAP_ERR_I2C_RX - I2C receive failed",
    4: "SNAP_ERR_SPI_RX - SPI receive failed",
    5: "SNAP_ERR_BUSY - Slave device busy (retry needed)",
    6: "SNAP_ERR_INVALID_SIZE - Invalid data size",
    7: "SNAP_ERR_INVALID_HEADER - Invalid packet header",
    8: "SNAP_ERR_INVALID_LENGTH - Invalid payload length",
    9: "SNAP_ERR_TIMEOUT - Operation timeout",
    10: "SNAP_ERR_INVALID_CMD - Invalid command code",
    255: "SNAP_ERR_UNKNOWN - Unknown error"
}

OSC_CHUNK_BYTES = 32768      # 32 KB per chunk
BYTES_PER_SAMPLE = 2         # 16-bit samples

COMMAND_OS_START_CONTINUOUS = 7
COMMAND_OS_STOP_CONTINUOUS  = 8
COMMAND_OS_GET_CHUNK        = 9
COMMAND_OS_CONFIG           = 10

os_commands = {
    "os_start": 7,
    "os_stop": 8,
    "os_get_chunk": 9,
    "os_config": 10
}


def get_error_message(error_code):
    """Get human-readable error message for error code"""
    return error_codes.get(error_code, f"Unknown error code: {error_code}")

def send_command(command_code, payload=b'', serial_port= None):
    """
    Send a command with the new packet format

    Args:
        command_code: Command code (0-255)
        payload: Payload bytes (0-255 bytes)

    Returns:
        None

    Packet format: [start_marker][command][payload_length][payload...]
    """
    global ser
    if serial_port is None:
        serial_port = ser

    if serial_port is None or not serial_port.is_open:
        raise Exception("Serial port not open")

    # Validate payload size
    payload_len = len(payload)
    if payload_len > 255:
        raise ValueError("Payload too large (max 255 bytes)")

    # Build packet header
    header = bytes([
        PACKET_START_MARKER_REQUEST,  # 0xAA
        command_code,
        payload_len
    ])

    # Send header + payload
    packet = header + payload
    print(f"[{get_timestamp()}] [USB TX] Sending packet: cmd={command_code}, len={payload_len}, hex={packet.hex()}")
    serial_port.write(packet)
    serial_port.flush()

def read_response(timeout=2.0, serial_port = None):
    """
    Read a response with the new packet format

    Args:
        timeout: Timeout in seconds (default 2.0)

    Returns:
        tuple: (status_code, payload_bytes)

    Packet format: [start_marker][status][payload_length][payload...]

    Raises:
        Exception: If timeout, invalid header, or communication error
    """
    global ser
    if serial_port is None:
        serial_port = ser
    if serial_port is None or not serial_port.is_open:
        raise Exception("Serial port not open")

    # Save original timeout
    original_timeout = serial_port.timeout
    serial_port.timeout = timeout

    try:
        # Read 3-byte header
        header_bytes = serial_port.read(PACKET_HEADER_SIZE)

        if len(header_bytes) < PACKET_HEADER_SIZE:
            raise Exception(f"Timeout reading response header (got {len(header_bytes)} bytes)")

        start_marker = header_bytes[0]
        status_code = header_bytes[1]
        payload_length = header_bytes[2]

        print(f"[{get_timestamp()}] [USB RX] Response header: marker=0x{start_marker:02x}, status={status_code}, len={payload_length}")

        # Validate start marker
        if start_marker != PACKET_START_MARKER_RESPONSE:
            raise Exception(f"Invalid response start marker: 0x{start_marker:02x} (expected 0x{PACKET_START_MARKER_RESPONSE:02x})")

        # Read payload if present
        payload = b''
        if payload_length > 0:
            payload = serial_port.read(payload_length)
            if len(payload) < payload_length:
                raise Exception(f"Timeout reading payload (expected {payload_length}, got {len(payload)} bytes)")
            print(f"[{get_timestamp()}] [USB RX] Payload: {payload.hex()}")

        # Raise exception if status indicates error
        if status_code != 0:
            error_msg = get_error_message(status_code)
            raise Exception(f"Command failed: {error_msg}")

        return (status_code, payload)

    finally:
        # Restore original timeout
        serial_port.timeout = original_timeout

def get_timestamp():
    """Get current timestamp string"""
    return time.strftime("%Y-%m-%d %H:%M:%S.") + f"{int(time.time() * 1000) % 1000:03d}"

def flush_serial_buffers():
    """Flush any stale data from serial buffers before sending new command"""
    global ser
    if ser is None or not ser.is_open:
        return
    
    try:
        # First check how much data is waiting
        in_waiting = ser.in_waiting
        if in_waiting > 0:
            # Read the data before flushing so we can see what's being discarded
            stale_data = ser.read(in_waiting)
            print(f"[{get_timestamp()}] [USB LOG] Flushing {in_waiting} bytes from input buffer before command")
            print(f"[{get_timestamp()}] [USB LOG] Flushed data (hex): {stale_data[:100].hex()}")  # Show first 100 bytes
            print(f"[{get_timestamp()}] [USB LOG] Flushed data (ascii): {stale_data[:100].decode(errors='ignore')}")
            # If there's a lot of data, drain it aggressively
            if in_waiting > 100:
                print(f"[{get_timestamp()}] [USB LOG] Large amount of stale data detected, draining aggressively...")
                drain_all_data()
                return
        
        # Normal flush for small amounts
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Error during flush: {e}")
        # Ignore any errors during flush
        pass

def flush_on_timeout():
    """Flush buffers after a timeout to clear stale data"""
    global ser
    if ser is None or not ser.is_open:
        return
    
    try:
        in_waiting = ser.in_waiting
        if in_waiting > 0:
            # Read the data before flushing so we can see what's being discarded
            stale_data = ser.read(in_waiting)
            print(f"[{get_timestamp()}] [USB LOG] Flushing {in_waiting} bytes from input buffer after timeout")
            print(f"[{get_timestamp()}] [USB LOG] Flushed data (hex): {stale_data[:100].hex()}")  # Show first 100 bytes
            print(f"[{get_timestamp()}] [USB LOG] Flushed data (ascii): {stale_data[:100].decode(errors='ignore')}")
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Error during timeout flush: {e}")
        pass

def drain_all_data():
    """Aggressively drain all pending data from serial port
    
    Use this when you suspect data is still streaming (e.g., after LA operations)
    """
    global ser
    if ser is None or not ser.is_open:
        return
    
    try:
        total_drained = 0
        old_timeout = ser.timeout
        ser.timeout = 0.01  # Very short timeout to quickly check for data
        
        # Keep reading until nothing comes back
        first_chunk = True
        while True:
            chunk = ser.read(4096)  # Read large chunks
            if len(chunk) == 0:
                break
            if first_chunk and len(chunk) > 0:
                # Show first chunk of drained data
                print(f"[{get_timestamp()}] [USB LOG] First drained chunk (hex): {chunk[:100].hex()}")
                print(f"[{get_timestamp()}] [USB LOG] First drained chunk (ascii): {chunk[:100].decode(errors='ignore')}")
                first_chunk = False
            total_drained += len(chunk)
            
        ser.timeout = old_timeout
        
        if total_drained > 0:
            print(f"[{get_timestamp()}] [USB LOG] Drained {total_drained} bytes of stale data from serial port")
        
        # Also reset buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Error during drain: {e}")
        ser.timeout = old_timeout

def find_cdc_devices():
    """Find all CDC serial ports for the main board"""
    ports = serial.tools.list_ports.comports()
    matching_ports = []
    
    for port in ports:
        # Check if VID/PID match
        if port.vid == VID and port.pid == PID:
            try:
                print('open ser')
                test_ser = serial.Serial(port.device, 115200, timeout=0.15)
                send_command(COMMAND_PING, b"", serial_port=test_ser)
                status, response_bytes = read_response(timeout=2.0, serial_port=test_ser)  # Expect 'pong' payload
                print('close ser')
                test_ser.close()
                print(f"[{get_timestamp()}] [USB LOG] Received {len(response_bytes)} bytes: {response_bytes.hex() if len(response_bytes) > 0 else '(empty)'}")
                print(f"[{get_timestamp()}] [USB LOG] Response as string: '{response_bytes.decode(errors='ignore')}'")
                response = response_bytes.decode(errors="ignore") if len(response_bytes) > 0 else ""
            except Exception as ex:
                print(f"[{get_timestamp()}] [USB LOG] Exception occurred when scanning device {ex}")
                response = "failed"

            if response != "0pong":
                print(f"[{get_timestamp()}] [USB LOG] pinged device did not respond with '0pong', skipping")
                continue

            matching_ports.append({
                'device': port.device,
                'description': port.description
            })
            print(f"Found device: {port.device} - {port.description}, {port.location}")
    
    # Sort by device name to ensure consistent ordering (lower port number first)
    matching_ports.sort(key=lambda x: x['device'])
    return matching_ports

def initialize_device(port_name=None):
    """Initialize serial connection to CDC device
    
    Args:
        port_name: Specific port to connect to, or None to use first available
    """
    global ser
    
    if port_name is None:
        # Find first available device
        devices = find_cdc_devices()
        if len(devices) == 0:
            raise Exception("Device not found. Check USB connection.")
        port_name = devices[0]['device']
    
    # Open serial port
    # CDC uses virtual COM port, no need for specific baud rate but we set one anyway
    ser = serial.Serial(
        port=port_name,
        baudrate=115200,  # CDC ignores this but some systems require it
        timeout=1.0,       # 1 second read timeout
        write_timeout=1.0  # 1 second write timeout
    )
    
    print(f"Connected to port: {port_name}")
    
    # Give it a moment to settle
    time.sleep(0.1)
    
    # Flush any stale data
    flush_serial_buffers()
    
    return port_name

@app.route('/')
def index():
    """Serve the main HTML page"""
    return render_template('index.html')

@app.route('/api/ports', methods=['GET'])
def list_ports():
    """List available CDC ports"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO LIST PORTS")
    try:
        devices = find_cdc_devices()
        return jsonify({
            'success': True,
            'ports': devices,
            'count': len(devices)
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error listing ports: {str(e)}'
        })

@app.route('/api/connect', methods=['POST'])
def connect_device():
    """Connect to the USB CDC device"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO CONNECT")
    try:
        data = request.get_json() or {}
        port_name = data.get('port')  # Optional: specific port to connect to
        
        with device_lock:
            connected_port = initialize_device(port_name)
            port_info = f"Connected to {connected_port}"
        
        return jsonify({
            'success': True,
            'message': port_info,
            'port': connected_port
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Connection failed: {str(e)}'
        })

def pack_waveform_struct(waveform_type, amplitude, frequency, offset, phase, duty_cycle=None, upwards=None):
    """Pack WaveForm struct in C binary format

    Args:
        waveform_type: String name of waveform type (sine, square, triangle, ramp, pulse)
        amplitude: Float amplitude in Volts
        frequency: Float frequency in Hz
        offset: Float offset from 0V
        phase: Float phase in radians
        duty_cycle: Float duty cycle 0-100 (for pulse only)
        upwards: Bool direction flag (for ramp only)

    Returns:
        bytes: Packed struct (24 bytes)
    """
    # Get enum value for waveform type
    type_enum = waveform_map[waveform_type]

    # Start with enum (4 bytes, uint32 little-endian)
    packed = struct.pack('<I', type_enum)

    # Pack union data based on type
    if waveform_type in ['sine', 'square', 'triangle']:
        # struct Sine/Square/Triangle: 4 floats (Amplitude, Frequency, Offset, Phase)
        packed += struct.pack('<ffff', amplitude, frequency, offset, phase)
        # Pad to 20 bytes (largest union member)
        packed += b'\x00' * 4

    elif waveform_type == 'ramp':
        # struct Ramp: 4 floats + 1 bool (Amplitude, Frequency, Offset, Phase, Upwards)
        upwards_val = 1 if upwards else 0
        packed += struct.pack('<ffffB', amplitude, frequency, offset, phase, upwards_val)
        # Pad to 20 bytes
        packed += b'\x00' * 3

    elif waveform_type == 'pulse':
        # struct Pulse: 5 floats (Amplitude, Frequency, Offset, DutyCycle, Phase)
        packed += struct.pack('<fffff', amplitude, frequency, offset, duty_cycle, phase)

    return packed



def hard_flush_serial():
    ser.timeout = 0.010  # short non-blocking read
    while True:
        b = ser.read(1024)
        if not b:
            break

@app.route('/api/waveform/<waveform_type>', methods=['POST'])
def send_waveform(waveform_type):
    """Send waveform command via CDC"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO SEND WAVEFORM: {waveform_type}")
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    if waveform_type not in waveform_map:
        return jsonify({
            'success': False,
            'message': f'Unknown waveform type: {waveform_type}'
        })

    # Get parameters from JSON request
    data = request.get_json() or {}
    amplitude = data.get('amplitude', 1.0)
    frequency = data.get('frequency', 10000.0)
    offset = data.get('offset', 1.0)
    phase = data.get('phase', 0.0)
    duty_cycle = data.get('duty_cycle', 10.0)  # For pulse
    upwards = data.get('upwards', True)  # For ramp

    try:
        # Validate parameters
        try:
            amplitude = float(amplitude)
            frequency = float(frequency)
            offset = float(offset)
            phase = float(phase)
            duty_cycle = float(duty_cycle)
            upwards = bool(upwards)
        except ValueError as e:
            return jsonify({
                'success': False,
                'message': f'Invalid parameter type: {str(e)}'
            })

        with device_lock:
            # Flush any stale data from previous commands
            flush_serial_buffers()

            # Pack the WaveForm struct
            waveform_struct = pack_waveform_struct(
                waveform_type, amplitude, frequency, offset, phase, duty_cycle, upwards
            )

            print(f"[{get_timestamp()}] [USB LOG] Sending waveform command: {waveform_type}")
            print(f"[{get_timestamp()}] [USB LOG] Parameters: amp={amplitude}V, freq={frequency}Hz, offset={offset}V, phase={phase}rad")
            if waveform_type == 'pulse':
                print(f"[{get_timestamp()}] [USB LOG] Duty cycle: {duty_cycle}%")
            elif waveform_type == 'ramp':
                print(f"[{get_timestamp()}] [USB LOG] Direction: {'upwards' if upwards else 'downwards'}")

            # Send command using new packet format
            send_command(COMMAND_SET_WAVEFORM, waveform_struct)

            # Read response using new packet format
            status, payload = read_response(timeout=2.0)

            # Success - new packet format guarantees status == 0 here
            return jsonify({
                'success': True,
                'message': f'Successfully sent {waveform_type} waveform',
                'parameters': {
                    'amplitude': amplitude,
                    'frequency': frequency,
                    'offset': offset,
                    'phase': phase,
                    'duty_cycle': duty_cycle if waveform_type == 'pulse' else None,
                    'upwards': upwards if waveform_type == 'ramp' else None
                }
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

############################################################
######################  PSU ENDPOINTS  #####################
############################################################

def parse_pdo_entry(pdo_bytes, index):
    """Parse a 4-byte PDO entry into voltage/current specs

    Args:
        pdo_bytes: 4 bytes representing one PDO entry
        index: PDO slot number (0-12)

    Returns:
        dict with voltage, current, type info or None if invalid
    """
    if len(pdo_bytes) < 4:
        return None

    byte0 = pdo_bytes[0]
    byte1 = pdo_bytes[1]

    # Extract bitfields (bits 0-15 across byte0 and byte1)
    voltage_max = byte0  # bits 7:0
    field_9_8 = (byte1 >> 0) & 0x03  # bits 9:8 (peak_current or voltage_min)
    current_max = (byte1 >> 2) & 0x0F  # bits 13:10
    type_bit = (byte1 >> 6) & 0x01  # bit 14
    detect_bit = (byte1 >> 7) & 0x01  # bit 15

    # Skip invalid PDOs
    if detect_bit == 0:
        return None

    # Determine PDO type
    pdo_type = "Fixed" if type_bit == 0 else ("PPS" if index < 7 else "AVS")

    if pdo_type == "Fixed":
        voltage_v = voltage_max * 0.1
        current_a = current_max * 0.25
        voltage_min_v = None
    else:
        # PPS/AVS: voltage in 100mV units, current via lookup table
        voltage_v = voltage_max * 0.1
        voltage_min_v = field_9_8 * 0.1
        current_a = PPS_CURRENT_MAP.get(current_max, 0.0)

    power_w = voltage_v * current_a

    return {
        "slot": index,
        "type": pdo_type,
        "voltage_max": round(voltage_v, 2),
        "voltage_min": round(voltage_min_v, 2) if voltage_min_v is not None else None,
        "current_max": round(current_a, 2),
        "power_max": round(power_w, 2),
        "valid": True
    }

@app.route('/api/psu/capabilities', methods=['POST'])
def psu_query_capabilities():
    """Query USB PD capabilities from connected adapter"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO QUERY PD CAPABILITIES")

    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    try:
        with device_lock:
            flush_serial_buffers()

            # Send COMMAND_PD_QUERY_CAPABS using new packet format
            command_code = psu_commands['psu_query_capabs']
            print(f"[{get_timestamp()}] [USB LOG] Sending PD query capabs command (code: {command_code})")

            # Send command (no payload)
            send_command(command_code, b'')

            # Read response with new packet format (use longer timeout for I2C query)
            status, response_bytes = read_response(timeout=2.0)

            print(f"[{get_timestamp()}] [USB LOG] Received {len(response_bytes)} bytes")

            # Parse PDO entries (4 bytes each)
            capabilities = []
            num_entries = len(response_bytes) // 4
            for i in range(num_entries):
                offset = i * 4
                pdo_bytes = response_bytes[offset:offset+4]

                if len(pdo_bytes) >= 4:
                    pdo = parse_pdo_entry(pdo_bytes, i)
                    if pdo is not None:  # Only include valid PDOs
                        capabilities.append(pdo)

            print(f"[{get_timestamp()}] [USB LOG] Parsed {len(capabilities)} valid PDO entries")

            return jsonify({
                'success': True,
                'message': f'Found {len(capabilities)} power profile(s)',
                'capabilities': capabilities,
                'num_profiles': len(capabilities)
            })

    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception: {e}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

@app.route('/api/psu/status', methods=['POST'])
def psu_query_status():
    """Query PSU status (voltage, current, temp)"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO QUERY PSU STATUS")

    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    try:
        with device_lock:
            flush_serial_buffers()

            # Send COMMAND_PD_QUERY_STATUS using new packet format
            command_code = psu_commands['psu_query_status']
            print(f"[{get_timestamp()}] [USB LOG] Sending PD query status command (code: {command_code})")

            # Send command (no payload)
            send_command(command_code, b'')

            # Read response with new packet format
            status, response_bytes = read_response(timeout=2.0)

            print(f"[{get_timestamp()}] [USB LOG] Received {len(response_bytes)} bytes for status struct")

            # Unpack the PSUStatus struct (24 bytes total)
            # struct: 5 floats (20 bytes) + 1 byte flags + 3 bytes padding = 24 bytes
            status_data = struct.unpack('<fffffBBBB', response_bytes)

            req_current_lim = status_data[0]
            req_voltage = status_data[1]
            current_actual = status_data[2]
            voltage_actual = status_data[3]
            temp = status_data[4]
            bitfield_byte = status_data[5]

            # Extract bitfields
            selected_profile = bitfield_byte & 0x0F  # Lower 4 bits
            f5v_en = (bitfield_byte >> 4) & 0x01
            f3v_en = (bitfield_byte >> 5) & 0x01

            status_dict = {
                "req_current_lim": round(req_current_lim, 3),
                "req_voltage": round(req_voltage, 3),
                "current_actual": round(current_actual, 3),
                "voltage_actual": round(voltage_actual, 3),
                "temp_c": round(temp, 2),
                "selected_profile": selected_profile,
                "f5v_enabled": bool(f5v_en),
                "f3v_enabled": bool(f3v_en)
            }

            print(f"[{get_timestamp()}] [USB LOG] Parsed PSU Status: {status_dict}")

            return jsonify({
                'success': True,
                'message': 'Successfully queried PSU status',
                'status': status_dict
            })

    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception: {e}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

@app.route('/api/psu/on', methods=['POST'])
def psu_start():
    """Enable PSU Output"""
    return send_psu_command('psu_on')

@app.route('/api/psu/off', methods=['POST'])
def psu_stop():
    """Disable PSU Output"""
    return send_psu_command('psu_off')

@app.route('/api/psu/alive', methods=['POST'])
def psu_alive():
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO QUERY PSU LIFE")

    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    try:
        with device_lock:
            flush_serial_buffers()

            # Send COMMAND_PD_QUERY_ALIVE using new packet format
            command_code = psu_commands['psu_query_alive']
            print(f"[{get_timestamp()}] [USB LOG] Sending PD query alive command (code: {command_code})")

            # Send command (no payload)
            send_command(command_code, b'')

            # Read response with new packet format (payload is 1 byte: usbConn boolean)
            status, response_bytes = read_response(timeout=2.0)

            # Extract usbConn boolean from payload
            usb_connected = (response_bytes[0] == 1) if len(response_bytes) >= 1 else False

            return jsonify({
                'success': True,
                'message': 'Successfully queried PSU alive status',
                'status': usb_connected
            })

    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception: {e}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

@app.route('/api/psu/config', methods=['POST'])
def psu_config():
    """Configure power supply voltage and current"""
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    data = request.get_json()
    voltage = data.get('voltage')
    current_limit = data.get('current_limit', 2.0)  # Default to 2.0A if not specified
    pdslot = data.get('pdslot', 0)
    f3Ven = data.get('f3Ven', 0)
    f5Ven = data.get('f5Ven', 0)

    if voltage is None:
        return jsonify({
            'success': False,
            'message': 'Voltage parameter required'
        })

    try:
        voltage = float(voltage)
        current_limit = float(current_limit)
    except ValueError:
        return jsonify({
            'success': False,
            'message': 'Voltage and current_limit must be floats'
        })

    try:
        with device_lock:
            # Flush any stale data from previous commands
            flush_serial_buffers()

            # Pack PSUOutputCommand struct:
            # float reqCurrentLim
            # float reqVoltage
            # uint8_t flags (selectedProfile:4, F5V_en:1, F3V_en:1, reserved:2)
            flags = ((f3Ven & 0b1) << 5) | ((f5Ven & 0b1) << 4) | ((pdslot+1) & 0x0F)
            payload = struct.pack('<ffBxxx', current_limit, voltage, flags)

            print(f"[{get_timestamp()}] [USB LOG] Sending PD config: voltage={voltage}V, current={current_limit}A, flags=0x{flags:02x}")

            # Send COMMAND_PD_REQUEST_OUTPUT using new packet format
            send_command(psu_commands['psu_config'], payload)  # Command code 10 = COMMAND_PD_REQUEST_OUTPUT

            # Read response using new packet format
            status, response_payload = read_response(timeout=2.0)

            # Success - new packet format guarantees status == 0 here
            return jsonify({
                'success': True,
                'message': f'Successfully configured voltage to {voltage}V with {current_limit}A current limit'
            })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

############################################################

@app.route('/api/la/start', methods=['POST'])
def la_start():
    """Start logic analyzer"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO START LA")
    return send_la_command('la_start')

@app.route('/api/la/stop', methods=['POST'])
def la_stop():
    """Stop logic analyzer"""
    api_start_time = time.time()
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO STOP LA")
    return send_la_command('la_stop', api_start_time=api_start_time)

@app.route('/api/la/get_chunk', methods=['POST'])
def la_get_chunk():
    """Get logic analyzer data chunk"""
    api_start_time = time.time()
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO GET LA CHUNK")
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    # Get num_chunks parameter from request
    data = request.get_json() or {}
    num_chunks = data.get('num_chunks', 50)  # Default to 50 if not specified

    try:
        num_chunks = int(num_chunks)
        if num_chunks <= 0:
            return jsonify({
                'success': False,
                'message': 'num_chunks must be a positive integer'
            })
    except (ValueError, TypeError):
        return jsonify({
            'success': False,
            'message': 'num_chunks must be a valid integer'
        })

    try:
        # Timing: waiting for lock
        lock_wait_start = time.time()
        with device_lock:
            lock_acquired_time = time.time()
            lock_wait_ms = (lock_acquired_time - lock_wait_start) * 1000
            # Flush any stale data from previous commands
            flush_start = time.time()
            flush_serial_buffers()
            flush_time_ms = (time.time() - flush_start) * 1000

            # Pack num_chunks (uint32_t, little-endian) as payload
            payload = struct.pack("<I", num_chunks)
            print(f"[{get_timestamp()}] [USB LOG] Sending la_get_chunk command (num_chunks: {num_chunks})")
            usb_send_time = time.time()

            # Send command using new packet format
            send_command(la_commands['la_get_chunk'], payload)
            write_time_ms = (time.time() - usb_send_time) * 1000

            # Read packet header to get total byte count, then stream that many bytes
            original_timeout = ser.timeout
            ser.timeout = 30.0  # extended timeout for acquisition + transfer

            try:
                header_bytes = ser.read(PACKET_HEADER_SIZE)
                if len(header_bytes) < PACKET_HEADER_SIZE:
                    raise Exception(f"Timeout reading response header (got {len(header_bytes)} bytes)")

                start_marker, status_code, payload_length = header_bytes[0], header_bytes[1], header_bytes[2]
                if start_marker != PACKET_START_MARKER_RESPONSE:
                    raise Exception(f"Invalid response start marker: 0x{start_marker:02x} (expected 0x{PACKET_START_MARKER_RESPONSE:02x})")

                meta_payload = b""
                if payload_length > 0:
                    meta_payload = ser.read(payload_length)
                    if len(meta_payload) < payload_length:
                        raise Exception(f"Timeout reading metadata payload (expected {payload_length}, got {len(meta_payload)})")

                if status_code != 0:
                    error_msg = get_error_message(status_code)
                    raise Exception(f"Command failed: {error_msg}")

                total_bytes = struct.unpack("<I", meta_payload[:4])[0] if len(meta_payload) >= 4 else 0
                if total_bytes == 0:
                    raise Exception("Device returned zero-length chunk payload")

                initial_response_time = time.time()
                initial_response_ms = (initial_response_time - usb_send_time) * 1000

                # Read the full sample payload
                sample_data = bytearray()
                remaining = total_bytes
                while remaining > 0:
                    chunk = ser.read(min(4096, remaining))
                    if len(chunk) == 0:
                        raise Exception(f"Timeout reading chunk data (remaining {remaining} bytes)")
                    sample_data.extend(chunk)
                    remaining -= len(chunk)

                data_complete_time = time.time()
                data_transfer_ms = (data_complete_time - initial_response_time) * 1000
            finally:
                ser.timeout = original_timeout

            print(f"[{get_timestamp()}] [USB LOG] Total data received: {len(sample_data)} bytes")

            # Calculate total time
            total_time_ms = (time.time() - api_start_time) * 1000

            # Log timing breakdown
            print(f"[{get_timestamp()}] [TIMING] Lock wait: {lock_wait_ms:.2f}ms | Flush: {flush_time_ms:.2f}ms | Write: {write_time_ms:.2f}ms | Initial response: {initial_response_ms:.2f}ms | Data transfer: {data_transfer_ms:.2f}ms | Total: {total_time_ms:.2f}ms")

            return jsonify({
                'success': True,
                'message': f'Successfully received {len(sample_data)} bytes of sample data',
                'status': 'ok',
                'data_hex': sample_data[:1000].hex(),  # Only send first 1000 bytes as hex for logging
                'data_bytes': list(sample_data),  # Send ALL bytes for visualization
                'total_bytes': len(sample_data),
                'timing': {
                    'lock_wait_ms': round(lock_wait_ms, 2),
                    'flush_time_ms': round(flush_time_ms, 2),
                    'write_time_ms': round(write_time_ms, 2),
                    'initial_response_ms': round(initial_response_ms, 2),
                    'data_transfer_ms': round(data_transfer_ms, 2),
                    'total_time_ms': round(total_time_ms, 2)
                }
            })
        
    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception in la_get_chunk: {e}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

@app.route('/api/la/config', methods=['POST'])
def la_config():
    """Configure logic analyzer frequency"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO CONFIG LA")
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })
    
    data = request.get_json()
    frequency = data.get('frequency')
    
    if frequency is None:
        return jsonify({
            'success': False,
            'message': 'Frequency parameter required'
        })
    
    try:
        frequency = int(frequency)
    except ValueError:
        return jsonify({
            'success': False,
            'message': 'Frequency must be an integer'
        })
    
    try:
        with device_lock:
            # Flush any stale data from previous commands
            flush_serial_buffers()

            # Pack uint32_t frequency (little-endian) as payload
            payload = struct.pack('<I', frequency)
            print(f"[{get_timestamp()}] [USB LOG] Sending la_config command: frequency={frequency} Hz")

            # Send command using new packet format
            send_command(la_commands["la_config"], payload)

            # Read response using new packet format
            status, response_payload = read_response(timeout=2.0)

            # Success - new packet format guarantees status == 0 here
            return jsonify({
                'success': True,
                'message': f'Successfully configured frequency to {frequency} Hz'
            })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

@app.route('/api/ping', methods=['POST'])
def ping_device():
    api_start_time = time.time()
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO PING")
    """Send ping test"""
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    try:
        # Timing: waiting for lock
        lock_wait_start = time.time()
        with device_lock:
            lock_acquired_time = time.time()
            lock_wait_ms = (lock_acquired_time - lock_wait_start) * 1000

            # Flush any stale data from previous commands
            flush_start = time.time()
            flush_serial_buffers()
            flush_time_ms = (time.time() - flush_start) * 1000

            print(f"[{get_timestamp()}] [USB LOG] Sending ping command (code: {COMMAND_PING}) using packet format")
            usb_send_time = time.time()
            send_command(COMMAND_PING, b"")
            write_time_ms = (time.time() - usb_send_time) * 1000

            # Wait for response
            status, response_bytes = read_response(timeout=2.0)  # Expect 'pong' payload
            response_received_time = time.time()
            response_wait_ms = (response_received_time - usb_send_time) * 1000

            print(f"[{get_timestamp()}] [USB LOG] Received {len(response_bytes)} bytes: {response_bytes.hex() if len(response_bytes) > 0 else '(empty)'}")
            print(f"[{get_timestamp()}] [USB LOG] Response as string: '{response_bytes.decode(errors='ignore')}'")

            response = response_bytes.decode(errors="ignore") if len(response_bytes) > 0 else ""

        # Calculate total time
        total_time_ms = (time.time() - api_start_time) * 1000

        # Log timing breakdown
        print(f"[{get_timestamp()}] [TIMING] Lock wait: {lock_wait_ms:.2f}ms | Flush: {flush_time_ms:.2f}ms | Write: {write_time_ms:.2f}ms | Response: {response_wait_ms:.2f}ms | Total: {total_time_ms:.2f}ms")

        return jsonify({
            'success': True,
            'message': 'Ping response received',
            'response': response,
            'response_length': len(response_bytes),
            'response_hex': response_bytes.hex() if len(response_bytes) > 0 else '',
            'timing': {
                'lock_wait_ms': round(lock_wait_ms, 2),
                'flush_time_ms': round(flush_time_ms, 2),
                'write_time_ms': round(write_time_ms, 2),
                'response_wait_ms': round(response_wait_ms, 2),
                'total_time_ms': round(total_time_ms, 2)
            }
        })
    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception during ping: {e}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

@app.route('/api/usb/speed_test', methods=['POST'])
def usb_speed_test():
    """Test USB throughput by transferring data"""
    api_start_time = time.time()
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO TEST USB SPEED")

    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    try:
        data = request.get_json() or {}
        test_size_kb = data.get('test_size_kb', 512)  # Default 512KB

        print(f"[{get_timestamp()}] [USB LOG] Starting USB speed test with {test_size_kb}KB")

        # Timing: waiting for lock
        lock_wait_start = time.time()
        with device_lock:
            lock_acquired_time = time.time()
            lock_wait_ms = (lock_acquired_time - lock_wait_start) * 1000

            # Flush any stale data from previous commands
            flush_start = time.time()
            flush_serial_buffers()
            flush_time_ms = (time.time() - flush_start) * 1000

            # Send command with test size parameter
            print(f"[{get_timestamp()}] [USB LOG] Sending USB speed test command (code: {COMMAND_USB_SPEED_TEST})")
            cmd_send_time = time.time()
            payload = struct.pack('<I', test_size_kb)  # Little-endian uint32
            send_command(COMMAND_USB_SPEED_TEST, payload)
            write_time_ms = (time.time() - cmd_send_time) * 1000

            # Read response header with metadata
            status, meta_bytes = read_response(timeout=5.0)

            # Parse total bytes to expect from metadata
            if len(meta_bytes) < 4:
                raise Exception(f"Invalid metadata response: {len(meta_bytes)} bytes")

            total_bytes = struct.unpack('<I', meta_bytes[:4])[0]
            print(f"[{get_timestamp()}] [USB LOG] Expecting {total_bytes} bytes ({total_bytes/1024:.1f}KB)")

            # Read the actual data stream
            data_buffer = bytearray()
            download_start = time.time()

            while len(data_buffer) < total_bytes:
                bytes_remaining = total_bytes - len(data_buffer)
                chunk_size = min(4096, bytes_remaining)
                chunk = ser.read(chunk_size)

                if len(chunk) == 0:
                    # Timeout - no data received
                    raise Exception(f"Timeout reading data (received {len(data_buffer)}/{total_bytes} bytes)")

                data_buffer.extend(chunk)

                # Progress logging every 100KB
                if len(data_buffer) % (100 * 1024) == 0 or len(data_buffer) >= total_bytes:
                    progress_pct = (len(data_buffer) / total_bytes) * 100
                    print(f"[{get_timestamp()}] [USB LOG] Progress: {len(data_buffer)}/{total_bytes} bytes ({progress_pct:.1f}%)")

            download_duration = time.time() - download_start

            # Calculate throughput
            throughput_mbps = (total_bytes / 1024 / 1024) / download_duration

            print(f"[{get_timestamp()}] [USB LOG] Transfer complete: {total_bytes} bytes in {download_duration*1000:.2f}ms = {throughput_mbps:.2f} MB/s")

        # Calculate total time
        total_time_ms = (time.time() - api_start_time) * 1000

        # Log timing breakdown
        print(f"[{get_timestamp()}] [TIMING] Lock wait: {lock_wait_ms:.2f}ms | Flush: {flush_time_ms:.2f}ms | Write: {write_time_ms:.2f}ms | Download: {download_duration*1000:.2f}ms | Total: {total_time_ms:.2f}ms")

        return jsonify({
            'success': True,
            'message': 'USB speed test complete',
            'total_bytes': total_bytes,
            'test_size_kb': test_size_kb,
            'duration_ms': round(download_duration * 1000, 2),
            'throughput_mbps': round(throughput_mbps, 2),
            'timing': {
                'lock_wait_ms': round(lock_wait_ms, 2),
                'flush_time_ms': round(flush_time_ms, 2),
                'write_time_ms': round(write_time_ms, 2),
                'download_duration_ms': round(download_duration * 1000, 2),
                'total_time_ms': round(total_time_ms, 2)
            }
        })
    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception during USB speed test: {e}")
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

def send_la_command(command_name, api_start_time = None):
    """Helper function to send logic analyzer commands"""
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    try:
        print(f"[{get_timestamp()}] [USB LOG] waiting on device lock")
        with device_lock:
            print(f"[{get_timestamp()}] [USB LOG] received device lock")
            # Flush any stale data from previous commands
            flush_serial_buffers()

            command_code = la_commands[command_name]
            print(f"[{get_timestamp()}] [USB LOG] Sending LA command: {command_name} (code: {command_code})")
            usb_send_time = time.time()

            # Log time difference if api_start_time was provided
            if api_start_time is not None:
                time_diff_ms = (usb_send_time - api_start_time) * 1000
                print(f"[{get_timestamp()}] [USB LOG] Time from API request to USB send: {time_diff_ms:.2f} ms")

            # Send command using new packet format (no payload)
            send_command(command_code, b'')

            # Read response using new packet format
            status, payload = read_response(timeout=2.0)

            # Success - new packet format guarantees status == 0 here
            return jsonify({
                'success': True,
                'message': f'Successfully executed {command_name}'
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })
    

# oscope stuff
# ---------------- SCOPE HELPERS ----------------
@app.route('/api/scope/start', methods=['POST'])
def scope_start():
    """Start oscilloscope continuous capture"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO START SCOPE")
    return send_scope_command('os_start')

@app.route('/api/scope/stop', methods=['POST'])
def scope_stop():
    """Stop oscilloscope continuous capture"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO STOP SCOPE")
    return send_scope_command('os_stop')

@app.route('/api/scope/get_chunk', methods=['POST'])
def scope_get_chunk():
    """Get oscilloscope data chunk"""
    api_start_time = time.time()
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO GET OS CHUNK")

    global ser
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })


    # Get num_chunks parameter from request
    data = request.get_json() or {}
    num_chunks = data.get('num_chunks', 2)  # Default to 50 if not specified

    try:
        num_chunks = int(num_chunks)
        if num_chunks <= 0:
            return jsonify({
                'success': False,
                'message': 'num_chunks must be a positive integer'
            })
    except (ValueError, TypeError):
        return jsonify({
            'success': False,
            'message': 'num_chunks must be a valid integer'
        })

    try:
        # Timing: waiting for lock
        # Flush any stale data from previous commands
        flush_start = time.time()
        flush_serial_buffers()
        flush_time_ms = (time.time() - flush_start) * 1000.0

        # Pack num_chunks (uint32_t, little-endian) as payload
        payload = struct.pack("<I", num_chunks)
        #print(f"[{get_timestamp()}] [USB LOG] Sending os_get_chunk command (num_chunks: {num_chunks})")
        usb_send_time = time.time()

        # Send command using new packet format
        print(f"payload for os_get_chunk = [{payload}]")
        send_scope_command('os_get_chunk')
        write_time_ms = (time.time() - usb_send_time) * 1000.0

        ser.timeout = 0.050
        raw = ser.read(OSC_CHUNK_BYTES)

        return jsonify({
            'success': True,
            'message': f'Successfully received {len(raw)} bytes of sample data',
            'status': 'ok',
            'data_hex': list(raw)
        })

    except Exception as e:
        print(f"[{get_timestamp()}] [USB LOG] Exception in os_get_chunk: {e}")
        flush_on_timeout()  # optional: clear any half-sent junk
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })
    
    




@app.route('/api/scope/config', methods=['POST'])
def scopeConfig():
    """Configure oscilloscope sample rate"""
    print(f"[{get_timestamp()}] [API LOG] RECEIVED REQUEST TO CONFIG SCOPE")
    
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })

    data = request.get_json()
    frequency = data.get('frequency')

    if frequency is None:
        return jsonify({
            'success': False,
            'message': 'Frequency parameter required'
        })

    try:
        frequency = int(frequency)
    except ValueError:
        return jsonify({
            'success': False,
            'message': 'Frequency must be an integer'
        })

    try:
        with device_lock:
            flush_serial_buffers()
            payload = struct.pack('<I', frequency)
            print(f"[{get_timestamp()}] [USB LOG] Sending scope_config: frequency={frequency} Hz")

            send_command(os_commands["os_config"], payload)
            status, response_payload = read_response(timeout=2.0)

            return jsonify({
                'success': True,
                'message': f'Scope configured to {frequency} Hz'
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })
    
def send_scope_command(command_name, api_start_time = None):
    """Helper function to send oscilloscope commands"""
    if ser is None or not ser.is_open:
        return jsonify({
            'success': False,
            'message': 'Device not connected'
        })
    print(f"[LOCK DEBUG] device_lock locked? {device_lock.locked()}")
    try:
        print(f"[{get_timestamp()}] [USB LOG] waiting on device lock")
        with device_lock:
            print(f"[{get_timestamp()}] [USB LOG] received device lock")
            # Flush any stale data from previous commands
            flush_serial_buffers()
            print(f"[{get_timestamp()}] [USB LOG] right above the fucking os command thing")
            print(f"[DEBUG] command_name={command_name}")
            print(f"[DEBUG] os_commands keys: {os_commands.keys()}")

            command_code = os_commands[command_name]
            print(f"[{get_timestamp()}] [USB LOG] Sending OS command: {command_name} (code: {command_code})")
            usb_send_time = time.time()

            # Log time difference if api_start_time was provided
            if api_start_time is not None:
                time_diff_ms = (usb_send_time - api_start_time) * 1000
                print(f"[{get_timestamp()}] [USB LOG] Time from API request to USB send: {time_diff_ms:.2f} ms")

            # Send command using new packet format (no payload)
            send_command(command_code, b'')

            # Read response using new packet format
            status, payload = read_response(timeout=2.0)
            print(f"[{get_timestamp()}] [USB RX] OS Command '{command_name}' completed successfully.")
            # Success - new packet format guarantees status == 0 here
            return jsonify({
                'success': True,
                'message': f'Successfully executed {command_name}'
                
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error: {str(e)}'
        })

if __name__ == '__main__':
    print("Starting Flask web server...")
    print("\n=== Available Serial Ports ===")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  {port.device}: {port.description}")
        if port.vid and port.pid:
            print(f"    VID:PID = {port.vid:04X}:{port.pid:04X}")
    
    print(f"\nLooking for device with VID:PID = {VID:04X}:{PID:04X}")
    print("\nOpen http://localhost:4731 in your browser")
    app.run(debug=True, host='0.0.0.0', port=4731, threaded=True)