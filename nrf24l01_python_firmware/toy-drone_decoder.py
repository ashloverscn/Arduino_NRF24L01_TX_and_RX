#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Universal RC Protocol Sniffer & Auto-Decoder
Real-time unknown protocol detection and decoding for toy drones/RC
Methods: Promiscuous scanning, entropy analysis, pattern matching,
         protocol fingerprinting, adaptive decoding

Windows/Linux compatible with RTL-SDR, HackRF, or NRF24L01+ via serial
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import numpy as np
import threading
import queue
import time
import struct
import json
import re
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Tuple, Callable, Set
from enum import Enum, auto
from collections import defaultdict, deque, Counter
from datetime import datetime
import serial
import serial.tools.list_ports

# Optional SDR support - gracefully degrades if not installed
try:
    from rtlsdr import RtlSdr
    RTLSDR_AVAILABLE = True
except ImportError:
    RTLSDR_AVAILABLE = False

try:
    from gnuradio import gr, blocks, analog
    GNURADIO_AVAILABLE = True
except ImportError:
    GNURADIO_AVAILABLE = False


class DetectionMethod(Enum):
    """Protocol detection methodology"""
    PROMISCUOUS = auto()      # Listen on all channels/addresses
    ENTROPY = auto()          # Statistical entropy analysis
    PATTERN_MATCH = auto()    # Known pattern database
    CORRELATION = auto()      # Auto-correlation for periodicity
    CONSTELLATION = auto()    # I/Q constellation analysis
    ADAPTIVE = auto()         # Machine learning-style adaptation


@dataclass
class RawRFPacket:
    """Raw captured RF packet before decoding"""
    timestamp: float
    data: bytes                    # Raw bytes from SDR/NRF24
    rssi: Optional[int] = None     # Signal strength
    channel: int = 0               # RF channel/frequency
    sample_rate: float = 0.0       # Capture sample rate
    metadata: Dict = field(default_factory=dict)


@dataclass  
class DecodedProtocol:
    """Auto-discovered protocol structure"""
    name: str = "Unknown"
    confidence: float = 0.0
    
    # Physical layer
    frequency_mhz: float = 2400.0
    channel_spacing_mhz: float = 1.0
    data_rate_kbps: float = 250.0
    modulation: str = "GFSK"       # GFSK, FSK, OQPSK, etc.
    
    # Packet structure (discovered)
    preamble_pattern: bytes = field(default_factory=bytes)
    preamble_len: int = 0
    address_len: int = 0
    address: bytes = field(default_factory=bytes)
    payload_len: int = 0
    payload_len_fixed: bool = True
    crc_len: int = 0
    crc_type: str = "unknown"      # checksum, xor, crc8, crc16
    
    # Channel map (discovered through analysis)
    channel_map: List[int] = field(default_factory=list)
    
    # Timing
    packet_interval_ms: float = 0.0
    hop_interval_ms: float = 0.0
    
    # Decoded channel positions (auto-discovered)
    throttle_pos: Optional[Tuple[int, int]] = None      # (byte, bit, width)
    rudder_pos: Optional[Tuple[int, int, int]] = None
    elevator_pos: Optional[Tuple[int, int, int]] = None
    aileron_pos: Optional[Tuple[int, int, int]] = None


@dataclass
class UniversalPacket:
    """Fully decoded packet from any protocol"""
    timestamp: float
    raw: bytes
    protocol: DecodedProtocol
    
    # Normalized channels (-100 to +100)
    throttle: int = 0
    rudder: int = 0
    elevator: int = 0
    aileron: int = 0
    
    # Discovered switches
    switches: Dict[str, bool] = field(default_factory=dict)
    
    # Raw discovered fields
    fields: Dict[str, any] = field(default_factory=dict)
    
    # RF metadata
    rssi: Optional[int] = None
    channel: int = 0


class EntropyAnalyzer:
    """Statistical analysis for protocol discovery"""
    
    def __init__(self, window_size: int = 1000):
        self.window_size = window_size
        self.byte_histogram = np.zeros(256, dtype=np.int64)
        self.packet_sizes = deque(maxlen=window_size)
        self.timing_intervals = deque(maxlen=window_size)
        self.last_time = 0.0
        
        # Pattern detection
        self.preamble_candidates: Dict[bytes, int] = defaultdict(int)
        self.address_candidates: Dict[bytes, int] = defaultdict(int)
        self.fixed_positions: Set[int] = set()
        
    def analyze_packet(self, packet: RawRFPacket):
        """Add packet to analysis window"""
        data = packet.data
        
        # Size analysis
        self.packet_sizes.append(len(data))
        
        # Timing analysis
        if self.last_time > 0:
            interval = (packet.timestamp - self.last_time) * 1000  # ms
            self.timing_intervals.append(interval)
        self.last_time = packet.timestamp
        
        # Byte frequency (skip first few bytes if variable preamble)
        if len(data) >= 5:
            for b in data[2:]:  # Skip potential preamble
                self.byte_histogram[b] += 1
        
        # Preamble detection (first 1-4 bytes that are constant)
        if len(data) >= 4:
            for plen in [1, 2, 3, 4]:
                preamble = data[:plen]
                self.preamble_candidates[preamble] += 1
        
        # Address detection (bytes 2-6 that show low entropy)
        if len(data) >= 6:
            addr = data[2:6]
            self.address_candidates[addr] += 1
        
        # Fixed position detection (bytes that rarely change)
        if len(data) >= 8:
            for i in range(min(len(data), 20)):
                # Track in separate per-position histogram
                pass  # Implemented in full analysis
    
    def get_entropy_profile(self) -> Dict:
        """Calculate entropy statistics"""
        total = np.sum(self.byte_histogram)
        if total == 0:
            return {}
        
        # Shannon entropy
        probs = self.byte_histogram / total
        probs = probs[probs > 0]  # Remove zeros
        entropy = -np.sum(probs * np.log2(probs))
        max_entropy = 8.0  # 8 bits
        
        # Packet size statistics
        sizes = list(self.packet_sizes)
        size_entropy = 0.0
        if sizes:
            size_counts = Counter(sizes)
            total_sizes = len(sizes)
            size_probs = [c/total_sizes for c in size_counts.values()]
            size_entropy = -sum(p * np.log2(p) for p in size_probs if p > 0)
        
        # Timing statistics
        intervals = list(self.timing_intervals)
        timing_stats = {}
        if intervals:
            timing_stats = {
                'mean_ms': np.mean(intervals),
                'std_ms': np.std(intervals),
                'min_ms': np.min(intervals),
                'max_ms': np.max(intervals),
                'periodicity': self._detect_periodicity(intervals)
            }
        
        # Find most common preamble
        best_preamble = max(self.preamble_candidates.items(), key=lambda x: x[1]) if self.preamble_candidates else (b'', 0)
        best_addr = max(self.address_candidates.items(), key=lambda x: x[1]) if self.address_candidates else (b'', 0)
        
        return {
            'byte_entropy': entropy,
            'entropy_ratio': entropy / max_entropy,
            'size_entropy': size_entropy,
            'common_sizes': Counter(self.packet_sizes).most_common(5),
            'timing': timing_stats,
            'likely_preamble': (best_preamble[0].hex(), best_preamble[1]),
            'likely_address': (best_addr[0].hex(), best_addr[1]),
            'high_entropy': entropy > 7.0,  # Likely encrypted or random
            'low_entropy': entropy < 4.0,  # Likely fixed structure
        }
    
    def _detect_periodicity(self, intervals: List[float]) -> Optional[float]:
        """Detect periodic packet transmission using autocorrelation"""
        if len(intervals) < 10:
            return None
        
        # Simple periodicity: look for common interval clusters
        rounded = [round(x, 1) for x in intervals]
        most_common = Counter(rounded).most_common(1)[0]
        if most_common[1] > len(intervals) * 0.3:  # >30% same interval
            return most_common[0]
        return None


class PromiscuousScanner:
    """
    NRF24L01+ Promiscuous Mode Scanner
    Uses open RX, no CRC check, wide address matching to catch everything
    """
    
    # NRF24 promiscuous settings
    PROMISCUOUS_ADDRESS = bytes([0x00, 0x00, 0x00, 0x00, 0x00])
    WIDE_ADDRESS = bytes([0xAA, 0xAA, 0xAA, 0xAA, 0xAA])  # Catch-all pattern
    
    def __init__(self, controller: 'NRF24Controller'):
        self.controller = controller
        self.scanning = False
        self.channels = list(range(0, 126, 2))  # 2.400-2.525 GHz step 2MHz
        self.current_ch_idx = 0
        self.dwell_time_ms = 100  # Time per channel
        
        # Capture results
        self.raw_packets: queue.Queue = queue.Queue()
        self.channel_hits: Dict[int, int] = defaultdict(int)
        
    def configure_promiscuous(self):
        """Configure NRF24 for promiscuous reception"""
        cmds = [
            "i",           # Initialize
            "N",           # Reset
            "r 0",         # Disable CRC (receive everything!)
            "J 05",        # 5-byte address
            "M 0A 05 00 00 00 00 00",  # Zero address (match all in promiscuous)
            "B 2",         # 250kbps for better sensitivity
            "T 3",         # Max power
            "E 0",         # Enable pipe 0
            "L 0 20",      # 32-byte max payload (capture all)
            "U 0 0",       # Disable auto-ack
        ]
        for cmd in cmds:
            self.controller.send(cmd)
            time.sleep(0.05)
    
    def start_scan(self):
        """Start channel hopping scan"""
        self.scanning = True
        self.configure_promiscuous()
        
        def scan_loop():
            while self.scanning:
                ch = self.channels[self.current_ch_idx]
                self.controller.send(f"C {ch:02X}")
                self.controller.send("X 2")  # RX mode
                
                # Collect for dwell time
                start = time.time()
                while time.time() - start < self.dwell_time_ms / 1000.0:
                    # Read any available data
                    resp = self.controller.send("G 20", wait_for_response=True, timeout=0.1)
                    if resp and not resp.startswith("ERR"):
                        # Parse DATA line if present
                        if "DATA" in resp:
                            hex_data = resp.split("DATA")[1].strip()
                            try:
                                raw = bytes.fromhex(hex_data.replace(" ", ""))
                                self.raw_packets.put(RawRFPacket(
                                    timestamp=time.time(),
                                    data=raw,
                                    channel=ch,
                                    rssi=self._estimate_rssi(raw)
                                ))
                                self.channel_hits[ch] += 1
                            except ValueError:
                                pass
                    time.sleep(0.001)
                
                # Move to next channel
                self.current_ch_idx = (self.current_ch_idx + 1) % len(self.channels)
        
        self.scan_thread = threading.Thread(target=scan_loop, daemon=True)
        self.scan_thread.start()
    
    def _estimate_rssi(self, raw: bytes) -> int:
        """Estimate RSSI from packet quality (simple heuristic)"""
        # In real implementation, use NRF24 RPD register
        return max(0, 255 - len(raw))  # Placeholder
    
    def stop(self):
        self.scanning = False


class AdaptiveDecoder:
    """
    Adaptive protocol decoder that learns from captured packets
    Uses multiple strategies to reverse-engineer unknown protocols
    """
    
    def __init__(self):
        self.analyzer = EntropyAnalyzer()
        self.known_protocols: List[DecodedProtocol] = []
        self.active_decoder: Optional[DecodedProtocol] = None
        self.learning_mode = True
        
        # Channel position hypotheses
        self.channel_hypotheses: List[Dict] = []
        
        # Pattern database
        self.pattern_db = self._load_pattern_db()
    
    def _load_pattern_db(self) -> Dict:
        """Load known protocol patterns"""
        return {
            'bayang': {
                'preamble': bytes([0x42]*5),
                'size': 15,
                'channels': [0x0D, 0x4C, 0x36, 0x40],
                'interval_ms': 8.0,
            },
            'syma': {
                'preamble': bytes([0xAB, 0xCD, 0xEF]),
                'size': 10,
                'channels': [0x4C, 0x30, 0x48],  # X5C channels
                'interval_ms': 4.0,
            },
            'cx10': {
                'preamble': bytes([0x55, 0xAA, 0x5A, 0xA5]),
                'size': 15,
                'channels': [0x4C, 0x30, 0x48, 0x60],
                'interval_ms': 6.0,
            },
            'v2x2': {
                'preamble': bytes([0x66, 0x88, 0x99, 0xAA, 0xBB]),
                'size': 16,
                'channels': [0x4C, 0x30, 0x48],
                'interval_ms': 4.0,
            },
        }
    
    def process_packet(self, raw_pkt: RawRFPacket) -> Optional[UniversalPacket]:
        """Process raw packet through adaptive decoding pipeline"""
        data = raw_pkt.data
        
        # Step 1: Update statistics
        self.analyzer.analyze_packet(raw_pkt)
        
        # Step 2: Try known protocol detection
        detected = self._detect_known_protocol(data)
        if detected:
            return self._decode_known(data, detected, raw_pkt)
        
        # Step 3: Try structure inference if in learning mode
        if self.learning_mode:
            inferred = self._infer_structure(data)
            if inferred:
                return self._decode_inferred(data, inferred, raw_pkt)
        
        # Step 4: Return raw with best-effort field extraction
        return self._decode_raw(data, raw_pkt)
    
    def _detect_known_protocol(self, data: bytes) -> Optional[DecodedProtocol]:
        """Match against known protocol database"""
        profile = self.analyzer.get_entropy_profile()
        
        # Check timing patterns
        timing = profile.get('timing', {})
        interval = timing.get('mean_ms', 0)
        
        for name, pattern in self.pattern_db.items():
            score = 0.0
            
            # Size match
            if len(data) == pattern['size']:
                score += 0.3
            
            # Preamble match
            if len(data) >= len(pattern['preamble']):
                if data[:len(pattern['preamble'])] == pattern['preamble']:
                    score += 0.4
            
            # Timing match (if available)
            if interval > 0 and abs(interval - pattern['interval_ms']) < 2.0:
                score += 0.2
            
            if score > 0.5:
                proto = DecodedProtocol(
                    name=name,
                    confidence=score,
                    payload_len=pattern['size'],
                    preamble_pattern=pattern['preamble'],
                    preamble_len=len(pattern['preamble']),
                    channel_map=pattern['channels']
                )
                return proto
        
        return None
    
    def _infer_structure(self, data: bytes) -> Optional[DecodedProtocol]:
        """Infer protocol structure from statistical analysis"""
        profile = self.analyzer.get_entropy_profile()
        
        if profile.get('high_entropy', False):
            return None  # Too random, likely encrypted
        
        # Infer packet structure
        proto = DecodedProtocol(
            name="Inferred_" + datetime.now().strftime("%H%M%S"),
            confidence=0.5,
            payload_len=len(data)
        )
        
        # Detect preamble from most common prefix
        likely_preamble = profile.get('likely_preamble', (None, 0))
        if likely_preamble[1] > 10:  # At least 10 occurrences
            preamble_bytes = bytes.fromhex(likely_preamble[0])
            proto.preamble_pattern = preamble_bytes
            proto.preamble_len = len(preamble_bytes)
        
        # Detect address
        likely_addr = profile.get('likely_address', (None, 0))
        if likely_addr[1] > 10:
            addr_bytes = bytes.fromhex(likely_addr[0])
            proto.address = addr_bytes
            proto.address_len = len(addr_bytes)
        
        # Infer CRC from last bytes
        if len(data) >= 3:
            # Check if last byte is simple checksum
            calc_sum = sum(data[:-1]) & 0xFF
            calc_xor = 0
            for b in data[:-1]:
                calc_xor ^= b
            
            if calc_sum == data[-1]:
                proto.crc_len = 1
                proto.crc_type = "checksum"
            elif calc_xor == data[-1]:
                proto.crc_len = 1
                proto.crc_type = "xor"
        
        # Attempt channel position inference
        self._infer_channel_positions(data, proto)
        
        return proto
    
    def _infer_channel_positions(self, data: bytes, proto: DecodedProtocol):
        """Use heuristics to find stick positions in packet"""
        if len(data) < 6:
            return
        
        # Hypothesis 1: 8-bit channels at fixed positions (common in simple protocols)
        # Look for bytes that change smoothly (stick movement)
        
        # Hypothesis 2: 10/16-bit packed channels (Bayang-style)
        # Look for pairs of bytes that correlate
        
        # Hypothesis 3: MSB indicates sign (Hubsan-style)
        
        # For now, store positions for manual analysis
        proto.throttle_pos = (0, 0, 8)   # byte 0, bit 0, 8 bits - default guess
        proto.rudder_pos = (1, 0, 8)
        proto.elevator_pos = (2, 0, 8)
        proto.aileron_pos = (3, 0, 8)
    
    def _decode_known(self, data: bytes, proto: DecodedProtocol, raw: RawRFPacket) -> UniversalPacket:
        """Decode using known protocol structure"""
        pkt = UniversalPacket(
            timestamp=raw.timestamp,
            raw=data,
            protocol=proto,
            rssi=raw.rssi,
            channel=raw.channel
        )
        
        # Apply protocol-specific decoding
        if proto.name == 'bayang':
            self._decode_bayang(data, pkt)
        elif proto.name == 'syma':
            self._decode_syma(data, pkt)
        elif proto.name == 'cx10':
            self._decode_cx10(data, pkt)
        elif proto.name == 'v2x2':
            self._decode_v2x2(data, pkt)
        
        return pkt
    
    def _decode_bayang(self, data: bytes, pkt: UniversalPacket):
        """Bayang protocol decoder"""
        if len(data) >= 5:
            ail_raw = (data[0] << 2) | ((data[1] >> 6) & 0x03)
            ele_raw = ((data[1] & 0x3F) << 4) | ((data[2] >> 4) & 0x0F)
            thr_raw = ((data[2] & 0x0F) << 6) | ((data[3] >> 2) & 0x3F)
            rud_raw = ((data[3] & 0x03) << 8) | data[4]
            
            pkt.aileron = self._scale(ail_raw, 0, 1023, 512)
            pkt.elevator = self._scale(ele_raw, 0, 1023, 512)
            pkt.throttle = self._scale(thr_raw, 0, 1023, 0)
            pkt.rudder = self._scale(rud_raw, 0, 1023, 512)
        
        if len(data) > 5:
            flags = data[5]
            pkt.switches = {
                'flip': bool(flags & 0x01),
                'rth': bool(flags & 0x02),
                'picture': bool(flags & 0x04),
                'video': bool(flags & 0x08),
                'headless': bool(flags & 0x10),
                'inverted': bool(flags & 0x20),
                'rate_high': bool(flags & 0x40),
            }
    
    def _decode_syma(self, data: bytes, pkt: UniversalPacket):
        """Syma X5C protocol decoder"""
        if len(data) >= 4:
            pkt.throttle = self._scale(data[0], 0, 255, 0)
            pkt.rudder = self._scale(data[1], 0, 255, 128)
            pkt.elevator = self._scale(data[2], 0, 255, 128)
            pkt.aileron = self._scale(data[3], 0, 255, 128)
        
        if len(data) > 7:
            flags = data[7]
            pkt.switches = {
                'flip': bool(flags & 0x01),
                'picture': bool(flags & 0x02),
                'video': bool(flags & 0x04),
                'headless': bool(flags & 0x08),
                'rate_high': bool(flags & 0x10),
                'led': bool(flags & 0x20),
            }
    
    def _decode_cx10(self, data: bytes, pkt: UniversalPacket):
        """Cheerson CX-10 decoder"""
        if len(data) >= 8:
            pkt.throttle = self._scale((data[0] << 8) | data[1], 0, 65535, 0)
            pkt.rudder = self._scale((data[2] << 8) | data[3], 0, 65535, 32768)
            pkt.elevator = self._scale((data[4] << 8) | data[5], 0, 65535, 32768)
            pkt.aileron = self._scale((data[6] << 8) | data[7], 0, 65535, 32768)
        
        if len(data) > 14:
            flags = data[14]
            pkt.switches = {
                'flip': bool(flags & 0x01),
                'rate_high': bool(flags & 0x02),
                'picture': bool(flags & 0x04),
                'video': bool(flags & 0x08),
                'headless': bool(flags & 0x10),
                'rth': bool(flags & 0x20),
            }
    
    def _decode_v2x2(self, data: bytes, pkt: UniversalPacket):
        """V2x2 / WLToys decoder"""
        if len(data) >= 4:
            pkt.throttle = self._scale(data[0], 0, 255, 0)
            pkt.rudder = self._scale(data[1], 0, 255, 128)
            pkt.elevator = self._scale(data[2], 0, 255, 128)
            pkt.aileron = self._scale(data[3], 0, 255, 128)
        
        if len(data) > 4:
            flags = data[4]
            pkt.switches = {
                'flip': bool(flags & 0x01),
                'picture': bool(flags & 0x02),
                'video': bool(flags & 0x04),
                'headless': bool(flags & 0x08),
                'rth': bool(flags & 0x10),
                'calibration': bool(flags & 0x20),
                'led': bool(flags & 0x40),
                'rate_high': bool(flags & 0x80),
            }
    
    def _decode_inferred(self, data: bytes, proto: DecodedProtocol, raw: RawRFPacket) -> UniversalPacket:
        """Decode using inferred structure"""
        pkt = UniversalPacket(
            timestamp=raw.timestamp,
            raw=data,
            protocol=proto,
            rssi=raw.rssi,
            channel=raw.channel
        )
        
        # Best-effort channel extraction using inferred positions
        if proto.throttle_pos:
            pkt.throttle = self._extract_field(data, proto.throttle_pos)
        if proto.rudder_pos:
            pkt.rudder = self._extract_field(data, proto.rudder_pos)
        if proto.elevator_pos:
            pkt.elevator = self._extract_field(data, proto.elevator_pos)
        if proto.aileron_pos:
            pkt.aileron = self._extract_field(data, proto.aileron_pos)
        
        return pkt
    
    def _decode_raw(self, data: bytes, raw: RawRFPacket) -> UniversalPacket:
        """Return packet with raw analysis only"""
        proto = DecodedProtocol(
            name="Unknown_Raw",
            confidence=0.0,
            payload_len=len(data)
        )
        
        return UniversalPacket(
            timestamp=raw.timestamp,
            raw=data,
            protocol=proto,
            rssi=raw.rssi,
            channel=raw.channel,
            fields={'raw_hex': data.hex(), 'length': len(data)}
        )
    
    def _scale(self, val: int, min_v: int, max_v: int, center: int) -> int:
        """Scale to -100 to +100"""
        centered = val - center
        max_dev = max(center - min_v, max_v - center)
        if max_dev == 0:
            return 0
        return int((centered / max_dev) * 100)
    
    def _extract_field(self, data: bytes, pos: Tuple[int, int, int]) -> int:
        """Extract bit field from packet"""
        byte_idx, bit_idx, width = pos
        if byte_idx >= len(data):
            return 0
        
        # Simple byte extraction (extend for bit-level)
        return data[byte_idx]


class NRF24Controller:
    """Serial communication for NRF24L01+"""
    
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.response_buffer = ""
    
    def connect(self, port: str, baud: int = 115200) -> bool:
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
    
    def send(self, cmd: str, wait: bool = False, timeout: float = 0.5) -> Optional[str]:
        if not self.ser:
            return None
        
        with self.lock:
            self.response_buffer = ""
        
        try:
            self.ser.write(f"{cmd}\n".encode())
            
            if wait:
                start = time.time()
                while time.time() - start < timeout:
                    if self.ser.in_waiting:
                        data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='replace')
                        self.response_buffer += data
                        if '\n' in self.response_buffer:
                            return self.response_buffer.strip()
                    time.sleep(0.01)
                return self.response_buffer.strip() if self.response_buffer else None
            
            return "Sent"
        except Exception as e:
            return f"Error: {e}"


class UniversalSnifferGUI:
    """Main GUI for Universal RC Protocol Sniffer"""
    
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Universal RC Protocol Sniffer - Unknown Protocol Decoder")
        self.root.geometry("1400x900")
        
        self.controller = NRF24Controller()
        self.scanner: Optional[PromiscuousScanner] = None
        self.decoder = AdaptiveDecoder()
        
        self.packets: List[UniversalPacket] = []
        self.packet_queue: queue.Queue = queue.Queue()
        
        self.setup_ui()
        self.refresh_ports()
        
        # Processing thread
        self.processing = True
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()
        
        # GUI update
        self.update_gui()
    
    def setup_ui(self):
        main = ttk.Frame(self.root, padding="10")
        main.pack(fill="both", expand=True)
        
        # === CONNECTION ===
        conn_frame = ttk.LabelFrame(main, text="Hardware Connection", padding="10")
        conn_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        self.conn_btn = ttk.Button(conn_frame, text="Connect & Start Sniff", command=self.toggle_sniff)
        self.conn_btn.grid(row=0, column=3, padx=10)
        
        ttk.Label(conn_frame, text="Mode:").grid(row=0, column=4, padx=(20, 5))
        self.mode_var = tk.StringVar(value="Promiscuous")
        ttk.Combobox(conn_frame, textvariable=self.mode_var,
                    values=["Promiscuous", "Fixed Channel", "Channel Hop", "Raw Dump"],
                    width=15, state="readonly").grid(row=0, column=5, padx=5)
        
        # === SCANNER CONFIG ===
        config_frame = ttk.LabelFrame(main, text="Scanner Configuration", padding="10")
        config_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(config_frame, text="Channel Range:").grid(row=0, column=0, padx=5)
        self.ch_start = ttk.Spinbox(config_frame, from_=0, to=125, width=5)
        self.ch_start.grid(row=0, column=1, padx=5)
        self.ch_start.set(0)
        ttk.Label(config_frame, text="to").grid(row=0, column=2)
        self.ch_end = ttk.Spinbox(config_frame, from_=0, to=125, width=5)
        self.ch_end.grid(row=0, column=3, padx=5)
        self.ch_end.set(125)
        
        ttk.Label(config_frame, text="Dwell (ms):").grid(row=0, column=4, padx=(20, 5))
        self.dwell_ms = ttk.Spinbox(config_frame, from_=10, to=1000, width=6)
        self.dwell_ms.grid(row=0, column=5, padx=5)
        self.dwell_ms.set(100)
        
        ttk.Label(config_frame, text="Data Rate:").grid(row=0, column=6, padx=(20, 5))
        self.rate_var = tk.StringVar(value="250K")
        ttk.Combobox(config_frame, textvariable=self.rate_var,
                    values=["250K", "1M", "2M"], width=6, state="readonly").grid(row=0, column=7, padx=5)
        
        ttk.Button(config_frame, text="Apply Config", command=self.apply_config).grid(row=0, column=8, padx=10)
        
        # === MAIN PANED ===
        paned = ttk.PanedWindow(main, orient="horizontal")
        paned.pack(fill="both", expand=True, pady=(0, 10))
        
        # Left: Packet list + stats
        left = ttk.Frame(paned)
        paned.add(left, weight=1)
        
        # Protocol stats
        stats_frame = ttk.LabelFrame(left, text="Protocol Detection", padding="5")
        stats_frame.pack(fill="x", pady=(0, 5))
        
        self.proto_label = ttk.Label(stats_frame, text="No protocol detected", font=('Consolas', 10))
        self.proto_label.pack(anchor="w")
        
        self.confidence_bar = ttk.Progressbar(stats_frame, maximum=100, length=200)
        self.confidence_bar.pack(fill="x", pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=6, width=40, font=('Consolas', 9),
                                 state="disabled", bg="#f0f0f0")
        self.stats_text.pack(fill="x")
        
        # Packet tree
        tree_frame = ttk.LabelFrame(left, text="Captured Packets", padding="5")
        tree_frame.pack(fill="both", expand=True)
        
        cols = ("Time", "Protocol", "Conf%", "CH", "Throttle", "Rudder", "Elev", "Ail", "RSSI", "Len")
        self.tree = ttk.Treeview(tree_frame, columns=cols, show="headings", height=15)
        for c in cols:
            self.tree.heading(c, text=c)
            w = 50 if c in ("CH", "RSSI", "Len", "Conf%") else 70
            if c == "Protocol":
                w = 120
            self.tree.column(c, width=w)
        
        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        tree_frame.columnconfigure(0, weight=1)
        tree_frame.rowconfigure(0, weight=1)
        
        self.tree.bind("<<TreeviewSelect>>", self.on_select)
        
        # Right: Details + hex + inferred structure
        right = ttk.PanedWindow(paned, orient="vertical")
        paned.add(right, weight=2)
        
        # Packet details
        details_frame = ttk.LabelFrame(right, text="Packet Details", padding="10")
        right.add(details_frame, weight=2)
        
        self.details = scrolledtext.ScrolledText(details_frame, wrap="word", font=('Consolas', 10),
                                                 state="disabled", bg="#1e1e1e", fg="#00ff00")
        self.details.pack(fill="both", expand=True)
        
        # Hex view with structure overlay
        hex_frame = ttk.LabelFrame(right, text="Hex View [Preamble|Address|Payload|CRC]", padding="10")
        right.add(hex_frame, weight=1)
        
        self.hex_view = scrolledtext.ScrolledText(hex_frame, wrap="none", font=('Consolas', 11),
                                                  state="disabled", bg="#1e1e1e", fg="#d4d4d4", height=8)
        self.hex_view.pack(fill="both", expand=True)
        
        # Inferred structure
        struct_frame = ttk.LabelFrame(right, text="Inferred Protocol Structure", padding="10")
        right.add(struct_frame, weight=1)
        
        self.struct_text = scrolledtext.ScrolledText(struct_frame, wrap="word", font=('Consolas', 10),
                                                     state="disabled", bg="#2d2d2d", fg="#cccccc", height=10)
        self.struct_text.pack(fill="both", expand=True)
        
        # === BOTTOM CONTROLS ===
        bottom = ttk.Frame(main)
        bottom.pack(fill="x")
        
        # Manual decode
        ttk.Label(bottom, text="Manual Hex:").pack(side="left", padx=5)
        self.manual_hex = ttk.Entry(bottom, width=60)
        self.manual_hex.pack(side="left", padx=5, fill="x", expand=True)
        self.manual_hex.bind("<Return>", lambda e: self.manual_decode())
        ttk.Button(bottom, text="Decode", command=self.manual_decode).pack(side="left", padx=5)
        
        # Action buttons
        btn_frame = ttk.Frame(main)
        btn_frame.pack(fill="x", pady=(10, 0))
        
        ttk.Button(btn_frame, text="Export All", command=self.export_all).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Save Protocol", command=self.save_protocol).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Load Protocol", command=self.load_protocol).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Clear", command=self.clear_all).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Force Decode As...", command=self.force_decode).pack(side="left", padx=5)
        
        # Status
        self.status = ttk.Label(main, text="Ready", relief="sunken", anchor="w")
        self.status.pack(fill="x", pady=(10, 0))
    
    def refresh_ports(self):
        ports = [(p.device, p.description) for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = [f"{p[0]} - {p[1]}" for p in ports]
        if ports:
            self.port_combo.set(f"{ports[0][0]} - {ports[0][1]}")
    
    def get_port(self) -> str:
        return self.port_var.get().split()[0]
    
    def apply_config(self):
        """Apply scanner configuration"""
        if self.scanner:
            start = int(self.ch_start.get())
            end = int(self.ch_end.get())
            self.scanner.channels = list(range(start, end + 1, 2))
            self.scanner.dwell_time_ms = int(self.dwell_ms.get())
            self.status.config(text=f"Config: Channels {start}-{end}, Dwell {self.scanner.dwell_time_ms}ms")
    
    def toggle_sniff(self):
        if self.scanner and self.scanner.scanning:
            self.stop_sniff()
        else:
            self.start_sniff()
    
    def start_sniff(self):
        port = self.get_port()
        if not port:
            messagebox.showerror("Error", "Select a COM port")
            return
        
        if not self.controller.connect(port):
            messagebox.showerror("Error", f"Cannot connect to {port}")
            return
        
        self.scanner = PromiscuousScanner(self.controller)
        self.apply_config()
        self.scanner.start_scan()
        
        self.conn_btn.config(text="Stop Sniffing")
        self.status.config(text=f"Sniffing on {port}...")
        
        # Start reader thread
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()
    
    def stop_sniff(self):
        if self.scanner:
            self.scanner.stop()
        self.controller.disconnect()
        self.conn_btn.config(text="Connect & Start Sniff")
        self.status.config(text="Stopped")
    
    def read_loop(self):
        """Read from scanner queue and feed to decoder"""
        while self.scanner and self.scanner.scanning:
            try:
                raw_pkt = self.scanner.raw_packets.get(timeout=0.1)
                decoded = self.decoder.process_packet(raw_pkt)
                if decoded:
                    self.packet_queue.put(decoded)
            except queue.Empty:
                continue
    
    def process_loop(self):
        """Background processing of decoded packets"""
        while self.processing:
            try:
                pkt = self.packet_queue.get(timeout=0.1)
                self.packets.append(pkt)
                
                # Keep only last 1000
                if len(self.packets) > 1000:
                    self.packets = self.packets[-1000:]
                
            except queue.Empty:
                continue
    
    def update_gui(self):
        """Update GUI with latest packets"""
        # Update tree (batch updates for performance)
        new_packets = [p for p in self.packets if not hasattr(p, '_displayed')]
        
        for pkt in new_packets:
            pkt._displayed = True
            
            flags_str = ""
            for name, val in pkt.switches.items():
                if val:
                    flags_str += name[0].upper()
            
            item = self.tree.insert("", "end", values=(
                f"{pkt.timestamp:.2f}",
                pkt.protocol.name,
                f"{pkt.protocol.confidence*100:.0f}",
                pkt.channel,
                f"{pkt.throttle:+4d}",
                f"{pkt.rudder:+4d}",
                f"{pkt.elevator:+4d}",
                f"{pkt.aileron:+4d}",
                pkt.rssi or 0,
                len(pkt.raw)
            ))
            
            # Store reference
            pkt._tree_item = item
            
            # Auto-scroll
            self.tree.see(item)
            
            # Limit tree size
            children = self.tree.get_children()
            if len(children) > 500:
                self.tree.delete(children[0])
        
        # Update protocol detection display
        if self.packets:
            recent = [p.protocol.name for p in self.packets[-100:]]
            counts = Counter(recent)
            dominant = counts.most_common(1)[0]
            
            self.proto_label.config(text=f"Detected: {dominant[0]} ({dominant[1]}/{len(recent)} packets)")
            self.confidence_bar['value'] = self.packets[-1].protocol.confidence * 100
            
            # Update stats
            profile = self.decoder.analyzer.get_entropy_profile()
            self.update_stats_text(profile)
        
        self.root.after(100, self.update_gui)
    
    def update_stats_text(self, profile: Dict):
        self.stats_text.config(state="normal")
        self.stats_text.delete("1.0", "end")
        
        if profile:
            lines = [
                f"Byte Entropy: {profile.get('byte_entropy', 0):.2f}/8.00",
                f"Entropy Ratio: {profile.get('entropy_ratio', 0):.2%}",
                f"Common Sizes: {profile.get('common_sizes', [])}",
                "",
                "Timing:",
            ]
            timing = profile.get('timing', {})
            if timing:
                lines.extend([
                    f"  Mean Interval: {timing.get('mean_ms', 0):.1f} ms",
                    f"  Periodicity: {timing.get('periodicity', 'None')}",
                ])
            
            preamble = profile.get('likely_preamble', (None, 0))
            if preamble[0]:
                lines.extend([
                    "",
                    f"Likely Preamble: {preamble[0]} (seen {preamble[1]}x)",
                ])
            
            addr = profile.get('likely_address', (None, 0))
            if addr[0]:
                lines.append(f"Likely Address: {addr[0]} (seen {addr[1]}x)")
            
            self.stats_text.insert("1.0", "\n".join(lines))
        
        self.stats_text.config(state="disabled")
    
    def on_select(self, event):
        selection = self.tree.selection()
        if not selection:
            return
        
        item = selection[0]
        # Find packet by tree item
        pkt = None
        for p in self.packets:
            if hasattr(p, '_tree_item') and p._tree_item == item:
                pkt = p
                break
        
        if not pkt:
            return
        
        # Update details
        self.update_details(pkt)
        self.update_hex_view(pkt)
        self.update_structure(pkt)
    
    def update_details(self, pkt: UniversalPacket):
        text = f"""=== {pkt.protocol.name} ===
Confidence: {pkt.protocol.confidence*100:.1f}%
Timestamp: {datetime.fromtimestamp(pkt.timestamp).strftime('%H:%M:%S.%f')[:-3]}
RF Channel: {pkt.channel}
RSSI: {pkt.rssi or 'N/A'}
Packet Rate: {pkt.protocol.packet_rate} Hz

--- CONTROL CHANNELS ---
Throttle:  {pkt.throttle:+4d}% {'[IDLE]' if pkt.throttle < 5 else '[MAX]' if pkt.throttle > 95 else ''}
Rudder:    {pkt.rudder:+4d}% {'[LEFT]' if pkt.rudder < -10 else '[RIGHT]' if pkt.rudder > 10 else '[CENTER]'}
Elevator:  {pkt.elevator:+4d}% {'[DOWN]' if pkt.elevator < -10 else '[UP]' if pkt.elevator > 10 else '[CENTER]'}
Aileron:   {pkt.aileron:+4d}% {'[LEFT]' if pkt.aileron < -10 else '[RIGHT]' if pkt.aileron > 10 else '[CENTER]'}

--- SWITCHES ---
"""
        for name, val in pkt.switches.items():
            text += f"{name:15s}: {'ON' if val else 'off'}\n"
        
        if pkt.fields:
            text += "\n--- EXTRA FIELDS ---\n"
            for k, v in pkt.fields.items():
                text += f"{k}: {v}\n"
        
        self.details.config(state="normal")
        self.details.delete("1.0", "end")
        self.details.insert("1.0", text)
        self.details.config(state="disabled")
    
    def update_hex_view(self, pkt: UniversalPacket):
        """Show hex with color-coded structure overlay"""
        raw = pkt.raw
        proto = pkt.protocol
        
        # Calculate structure boundaries
        pre_end = proto.preamble_len
        addr_end = pre_end + proto.address_len
        payload_end = len(raw) - proto.crc_len
        
        lines = []
        hex_str = raw.hex(' ').upper()
        
        # Header with structure markers
        header = "Preamble      | Address       | Payload"
        if proto.crc_len > 0:
            header += "       | CRC"
        lines.append(header)
        lines.append("-" * len(header))
        
        # Hex with spacing
        bytes_hex = hex_str.split()
        formatted = []
        for i, b in enumerate(bytes_hex):
            if i == pre_end:
                formatted.append("|")
            elif i == addr_end:
                formatted.append("|")
            elif i == payload_end and proto.crc_len > 0:
                formatted.append("|")
            formatted.append(b)
        
        lines.append(" ".join(formatted))
        
        # ASCII
        ascii_str = ""
        for b in raw:
            if 32 <= b < 127:
                ascii_str += chr(b)
            else:
                ascii_str += "."
        
        lines.append("")
        lines.append(f"ASCII: {ascii_str}")
        
        self.hex_view.config(state="normal")
        self.hex_view.delete("1.0", "end")
        self.hex_view.insert("1.0", "\n".join(lines))
        self.hex_view.config(state="disabled")
    
    def update_structure(self, pkt: UniversalPacket):
        """Show inferred protocol structure"""
        proto = pkt.protocol
        
        text = f"""=== INFERRED PROTOCOL STRUCTURE ===
Name: {proto.name}
Confidence: {proto.confidence*100:.1f}%

--- PHYSICAL LAYER ---
Frequency: {proto.frequency_mhz} MHz
Channel Spacing: {proto.channel_spacing_mhz} MHz
Data Rate: {proto.data_rate_kbps} kbps
Modulation: {proto.modulation}

--- PACKET STRUCTURE ---
Preamble: {proto.preamble_pattern.hex().upper() if proto.preamble_pattern else 'Not detected'} ({proto.preamble_len} bytes)
Address: {proto.address.hex().upper() if proto.address else 'Not detected'} ({proto.address_len} bytes)
Payload: {proto.payload_len} bytes ({'Fixed' if proto.payload_len_fixed else 'Variable'})
CRC: {proto.crc_type} ({proto.crc_len} bytes)

--- CHANNEL MAP ---
Hopping Channels: {proto.channel_map}

--- TIMING ---
Packet Interval: {proto.packet_interval_ms:.2f} ms
Hop Interval: {proto.hop_interval_ms:.2f} ms

--- DISCOVERED CHANNEL POSITIONS ---
Throttle:  {proto.throttle_pos or 'Unknown'}
Rudder:    {proto.rudder_pos or 'Unknown'}
Elevator:  {proto.elevator_pos or 'Unknown'}
Aileron:   {proto.aileron_pos or 'Unknown'}
"""
        
        self.struct_text.config(state="normal")
        self.struct_text.delete("1.0", "end")
        self.struct_text.insert("1.0", text)
        self.struct_text.config(state="disabled")
    
    def manual_decode(self):
        hex_str = self.manual_hex.get().strip()
        try:
            raw = bytes.fromhex(hex_str.replace(" ", ""))
            raw_pkt = RawRFPacket(timestamp=time.time(), data=raw, channel=0)
            decoded = self.decoder.process_packet(raw_pkt)
            
            if decoded:
                self.packets.append(decoded)
                self.packet_queue.put(decoded)
                self.status.config(text=f"Manual decode: {decoded.protocol.name}")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid hex: {e}")
    
    def force_decode(self):
        """Force re-decode selected packet with specific protocol"""
        # Would show dialog to select protocol
        pass
    
    def export_all(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                           filetypes=[("JSON", "*.json"), ("CSV", "*.csv"), ("All", "*.*")])
        if path:
            if path.endswith('.json'):
                data = {
                    'protocol': asdict(self.packets[-1].protocol) if self.packets else {},
                    'packets': [
                        {
                            'timestamp': p.timestamp,
                            'protocol': p.protocol.name,
                            'raw': p.raw.hex(),
                            'channels': {
                                'throttle': p.throttle,
                                'rudder': p.rudder,
                                'elevator': p.elevator,
                                'aileron': p.aileron,
                            },
                            'switches': p.switches,
                        }
                        for p in self.packets
                    ]
                }
                with open(path, 'w') as f:
                    json.dump(data, f, indent=2)
            else:
                # CSV export
                with open(path, 'w') as f:
                    f.write("timestamp,protocol,raw_hex,throttle,rudder,elevator,aileron,channel,rssi\n")
                    for p in self.packets:
                        f.write(f"{p.timestamp},{p.protocol.name},{p.raw.hex()},"
                               f"{p.throttle},{p.rudder},{p.elevator},{p.aileron},"
                               f"{p.channel},{p.rssi or ''}\n")
            
            self.status.config(text=f"Exported {len(self.packets)} packets to {path}")
    
    def save_protocol(self):
        """Save discovered protocol definition"""
        if not self.packets:
            return
        
        proto = self.packets[-1].protocol
        path = filedialog.asksaveasfilename(defaultextension=".json")
        if path:
            with open(path, 'w') as f:
                json.dump(asdict(proto), f, indent=2)
            self.status.config(text=f"Saved protocol to {path}")
    
    def load_protocol(self):
        """Load protocol definition"""
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            with open(path, 'r') as f:
                data = json.load(f)
            # Would load into decoder
            self.status.config(text=f"Loaded protocol from {path}")
    
    def clear_all(self):
        self.packets.clear()
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.details.config(state="normal")
        self.details.delete("1.0", "end")
        self.details.config(state="disabled")
        self.hex_view.config(state="normal")
        self.hex_view.delete("1.0", "end")
        self.hex_view.config(state="disabled")
        self.struct_text.config(state="normal")
        self.struct_text.delete("1.0", "end")
        self.struct_text.config(state="disabled")
        self.status.config(text="Cleared")
    
    def on_closing(self):
        self.processing = False
        self.stop_sniff()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = UniversalSnifferGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()