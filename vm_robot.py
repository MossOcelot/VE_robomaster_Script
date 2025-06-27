import socket
import struct
from src.robomaster import algo
from src.robomaster import protocol

# Create UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind to localhost on port 40923 (standard RoboMaster SDK port)
server_address = ('0.0.0.0', 5000)
server_socket.bind(server_address)

def decode_wheel_speed_payload(payload):
    """แปลง payload ของ wheel speed command เป็นค่าที่อ่านได้
    
    :param payload: payload bytes จาก wheel speed command (cmdid=0x20)
    :return: dict ที่มีค่าความเร็วของแต่ละล้อ
    """
    if len(payload) >= 8:
        # แปลง payload เป็นค่าความเร็วของล้อ 4 ล้อ (signed short, little endian)
        w1_spd, w2_spd, w3_spd, w4_spd = struct.unpack('<hhhh', payload[:8])
        
        return {
            "w1_speed": w1_spd,
            "w2_speed": w2_spd,
            "w3_speed": w3_spd,
            "w4_speed": w4_spd,
            "decoded_payload": f"W1:{w1_spd}, W2:{w2_spd}, W3:{w3_spd}, W4:{w4_spd}"
        }
    else:
        return {
            "error": f"Payload too short for wheel speed command: {len(payload)} bytes"
        }

# Create UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind to localhost on port 40923 (standard RoboMaster SDK port)
server_address = ('0.0.0.0', 5000)
server_socket.bind(server_address)

def create_sdk_connection_response(seq_id, cmdset, cmdid, state=0, config_ip='192.168.42.2'):
    """สร้าง response สำหรับ SDK Connection request
    
    :param seq_id: sequence ID จาก request
    :param cmdset: command set จาก request (0x3f)
    :param cmdid: command ID จาก request (0xd4)
    :param state: 0=accept, 1=reject(busy), 2=config_ip
    :param config_ip: IP address ที่จะให้ client ใช้
    """
    # สร้าง response payload
    payload = bytearray()
    payload.append(0)  # retcode (0 = success)
    payload.append(state)  # state
    
    if state == 2:  # ถ้าเป็น config state ให้ส่ง IP
        ip_parts = config_ip.split('.')
        for part in ip_parts:
            payload.append(int(part))
    
    # คำนวณความยาว message
    msg_len = 13 + len(payload)  # header(11) + cmdset(1) + cmdid(1) + payload + crc16(2)
    
    # สร้าง message buffer
    buf = bytearray(msg_len)
    
    # Header
    buf[0] = 0x55  # magic number
    buf[1] = msg_len & 0xff
    buf[2] = ((msg_len >> 8) & 0x3) | 4
    
    # CRC8 ของ header
    crc_h = algo.crc8_calc(buf[0:3])
    buf[3] = crc_h
    
    # Message fields
    buf[4] = 0x09  # sender (robot)
    buf[5] = 0x00  # receiver (SDK)
    buf[6] = seq_id & 0xff
    buf[7] = (seq_id >> 8) & 0xff
    buf[8] = 0x80  # attributes (is_ack = True)
    buf[9] = cmdset
    buf[10] = cmdid
    
    # Payload
    if len(payload) > 0:
        buf[11:11+len(payload)] = payload
    
    # CRC16 ของ message ทั้งหมด
    crc_m = algo.crc16_calc(buf[0:msg_len-2])
    struct.pack_into('<H', buf, msg_len-2, crc_m)
    
    return bytes(buf)

def handle_sdk_connection_request(data, client_address):
    """จัดการ SDK Connection request"""
    unpacked = unpack(data)

    if "error" in unpacked:
        print(f"Error unpacking data: {unpacked['error']}")
        return None
    
    # ตรวจสอบว่าเป็น SDK Connection request หรือไม่
    if unpacked["cmdset"] == 0x3f and unpacked["cmdid"] == 0xd4:
        print(f"Received SDK Connection request from {client_address}")
        print(f"Payload: {unpacked['payload'].hex()}")
        
        # แยก payload ของ SDK Connection request
        payload = unpacked["payload"]
        if len(payload) >= 10:
            connection_type = payload[0]
            protocol_type = payload[1]
            host = payload[2]
            ip_bytes = payload[3:7]
            port = struct.unpack('<H', payload[7:9])[0]
            
            ip_address = '.'.join(str(b) for b in ip_bytes)
            
            print(f"Connection type: {connection_type}")
            print(f"Protocol: {protocol_type}")
            print(f"Host: {host}")
            print(f"Requested IP: {ip_address}")
            print(f"Requested Port: {port}")
            
            # สร้าง response
            # state = 0: accept connection
            # state = 2: provide config IP
            response = create_sdk_connection_response(
                unpacked["seq_id"], 
                unpacked["cmdset"], 
                unpacked["cmdid"],
                state=2,  # ส่ง config IP
                config_ip='127.0.0.1'  # ให้ใช้ localhost
            )
            
            return response
    
    return None

def unpack(data: bytes):
    result = {}
    try:
        # ตรวจสอบความยาวขั้นต่ำ
        if len(data) < 13:
            raise ValueError("Data too short to unpack")

        # ตรวจสอบ Header
        if data[0] != 0x55:
            raise ValueError("Invalid start byte")

        # อ่านความยาวข้อมูล
        msg_len = data[1] + ((data[2] & 0x3) << 8)
        if len(data) != msg_len:
            raise ValueError("Length mismatch: expected {}, got {}".format(msg_len, len(data)))

        # ตรวจสอบ CRC8 ของ Header
        crc_h = algo.crc8_calc(data[0:3])
        if data[3] != crc_h:
            raise ValueError("Header CRC8 mismatch")

        # อ่านค่าพื้นฐาน
        sender = data[4]
        receiver = data[5]
        seq_id = data[6] | (data[7] << 8)
        attri = data[8]
        is_ack = bool(attri & (1 << 7))
        need_ack = bool(attri & (1 << 5))

        # ตรวจสอบ CRC16
        crc_msg_calc = algo.crc16_calc(data[0:msg_len - 2])
        crc_msg_recv = struct.unpack_from('<H', data, msg_len - 2)[0]
        if crc_msg_calc != crc_msg_recv:
            raise ValueError("Message CRC16 mismatch")

        # ดึง cmdset / cmdid และ payload
        cmdset = data[9]
        cmdid = data[10]
        payload = data[11:msg_len - 2]

        # เพิ่มการแปลง payload สำหรับ wheel speed command
        # decoded_info = {}
        # if cmdset == 0x3f and cmdid == 0x20:  # Wheel speed command
        #     decoded_info = decode_wheel_speed_payload(payload)

        # จัดรูปแบบผลลัพธ์
        result = {
            "length": msg_len,
            "sender": sender,
            "receiver": receiver,
            "seq_id": seq_id,
            "is_ack": is_ack,
            "need_ack": need_ack,
            "cmdset": cmdset,
            "cmdid": cmdid,
            "payload": payload,
            "raw": data
        }
        
        # # เพิ่มข้อมูลที่แปลงแล้วถ้ามี
        # if decoded_info:
        #     result["decoded"] = decoded_info

    except Exception as e:
        result["error"] = str(e)

    return result

print(f"Virtual Robot UDP server listening on {server_address[0]}:{server_address[1]}")

try:
    while True:
        # Receive data from client
        data, client_address = server_socket.recvfrom(1024)
        print(f"\nReceived from {client_address}: {data.hex()}")
        
        # แสดงข้อมูลที่ unpack แล้ว
        unpacked = unpack(data)
        print(f"Unpacked: {unpacked}")
        
        # แสดงข้อมูลที่แปลงแล้วถ้าเป็น wheel speed command
        if "decoded" in unpacked:
            print(f"Decoded wheel speeds: {unpacked['decoded']['decoded_payload']}")
        
        # จัดการ SDK Connection request
        response = handle_sdk_connection_request(data, client_address)
        
        if response:
            print(f"Sending response: {response.hex()}")
            server_socket.sendto(response, client_address)
        else:
            print("No response needed or error occurred")
        
except KeyboardInterrupt:
    print("\nServer shutting down...")
finally:
    server_socket.close()