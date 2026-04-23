#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ultimate NRF24L01+ Serial Control GUI
Windows-compatible with COM port selection
Supports all commands from the Arduino firmware v3.0
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import re
from datetime import datetime
from collections import deque


class NRF24Controller:
    """Serial communication handler for NRF24L01+"""
    
    def __init__(self):
        self.ser = None
        self.port = None
        self.baud = 115200
        self.connected = False
        self.read_thread = None
        self.running = False
        self.callback = None
        self.command_history = deque(maxlen=100)
        self.response_buffer = ""
        self.lock = threading.Lock()
    
    def list_ports(self):
        """List available COM ports on Windows"""
        ports = []
        for p in serial.tools.list_ports.comports():
            desc = f"{p.device} - {p.description}" if p.description else p.device
            ports.append((p.device, desc))
        return ports
    
    def connect(self, port, baud=115200, timeout=1):
        """Connect to specified COM port"""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=1
            )
            self.port = port
            self.baud = baud
            self.connected = True
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            return True
        except Exception as e:
            return f"Error: {str(e)}"
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        self.connected = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
    
    def _read_loop(self):
        """Background thread to read serial data"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='replace')
                    with self.lock:
                        self.response_buffer += data
                    if self.callback:
                        self.callback(data)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"Read error: {e}")
                break
    
    def send(self, command, wait_for_response=False, timeout=2):
        """Send command to Arduino"""
        if not self.connected or not self.ser:
            return "Not connected"
        
        with self.lock:
            self.response_buffer = ""
        
        cmd = command.strip()
        self.command_history.append((datetime.now().isoformat(), cmd))
        
        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
            self.ser.flush()
            
            if wait_for_response:
                start = time.time()
                while time.time() - start < timeout:
                    with self.lock:
                        if '\n' in self.response_buffer or '\r' in self.response_buffer:
                            resp = self.response_buffer.strip()
                            return resp
                    time.sleep(0.01)
                return "Timeout"
            return "Sent"
        except Exception as e:
            return f"Send error: {e}"
    
    def send_raw(self, data):
        """Send raw bytes"""
        if self.ser and self.ser.is_open:
            self.ser.write(data)
            self.ser.flush()
    
    def get_buffer(self):
        """Get current response buffer"""
        with self.lock:
            return self.response_buffer
    
    def clear_buffer(self):
        """Clear response buffer"""
        with self.lock:
            self.response_buffer = ""


class NRF24GUI:
    """Main GUI Application"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("NRF24L01+ Ultimate Control Panel")
        self.root.geometry("1200x800")
        self.root.minsize(1000, 700)
        
        # Windows DPI awareness
        try:
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
        except:
            pass
        
        self.controller = NRF24Controller()
        self.controller.callback = self.on_serial_data
        
        # Theme colors
        self.bg_color = "#f0f0f0"
        self.accent_color = "#2196F3"
        self.success_color = "#4CAF50"
        self.error_color = "#f44336"
        self.warning_color = "#FF9800"
        
        self.setup_styles()
        self.build_ui()
        self.refresh_ports()
        
        # Periodic GUI updates
        self.pending_data = ""
        self.update_queue()
    
    def setup_styles(self):
        """Configure ttk styles"""
        style = ttk.Style()
        style.theme_use('clam')
        
        style.configure('Accent.TButton', background=self.accent_color, foreground='white')
        style.configure('Success.TButton', background=self.success_color, foreground='white')
        style.configure('Danger.TButton', background=self.error_color, foreground='white')
        style.configure('Title.TLabel', font=('Segoe UI', 12, 'bold'))
        style.configure('Header.TLabel', font=('Segoe UI', 10, 'bold'))
        style.configure('Mono.TLabelframe', font=('Consolas', 9))
    
    def build_ui(self):
        """Build the complete user interface"""
        # Main container with padding
        main = ttk.Frame(self.root, padding="10")
        main.grid(row=0, column=0, sticky="nsew")
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(1, weight=1)
        
        # === TOP BAR: Connection ===
        self.build_connection_frame(main, 0, 0)
        
        # === LEFT PANEL: Quick Actions ===
        self.build_left_panel(main, 1, 0)
        
        # === RIGHT PANEL: Output & Advanced ===
        self.build_right_panel(main, 1, 1)
        
        # === BOTTOM: Status Bar ===
        self.build_status_bar(main, 2, 0, 2)
    
    def build_connection_frame(self, parent, row, col):
        """COM port connection controls"""
        frame = ttk.LabelFrame(parent, text="Serial Connection", padding="10")
        frame.grid(row=row, column=col, columnspan=2, sticky="ew", pady=(0, 10))
        frame.columnconfigure(1, weight=1)
        
        # Port selection
        ttk.Label(frame, text="COM Port:", style='Header.TLabel').grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=40, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, sticky="ew")
        
        ttk.Button(frame, text="Refresh", command=self.refresh_ports, width=10).grid(row=0, column=2, padx=5)
        
        # Baud rate
        ttk.Label(frame, text="Baud:", style='Header.TLabel').grid(row=0, column=3, padx=(20, 5))
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(frame, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200"], width=10, state="readonly")
        baud_combo.grid(row=0, column=4, padx=5)
        
        # Connect/Disconnect buttons
        self.connect_btn = ttk.Button(frame, text="Connect", command=self.toggle_connection, style='Accent.TButton')
        self.connect_btn.grid(row=0, column=5, padx=5)
        
        self.disconnect_btn = ttk.Button(frame, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=6, padx=5)
        
        # Connection status indicator
        self.status_canvas = tk.Canvas(frame, width=20, height=20, highlightthickness=0)
        self.status_canvas.grid(row=0, column=7, padx=10)
        self.status_indicator = self.status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="")
    
    def build_left_panel(self, parent, row, col):
        """Left side control panel with tabs"""
        left = ttk.Notebook(parent, width=400)
        left.grid(row=row, column=col, sticky="nsew", padx=(0, 10))
        
        # Tab 1: Quick Controls
        tab1 = ttk.Frame(left, padding="10")
        left.add(tab1, text="Quick Controls")
        self.build_quick_controls(tab1)
        
        # Tab 2: Register Access
        tab2 = ttk.Frame(left, padding="10")
        left.add(tab2, text="Registers")
        self.build_register_controls(tab2)
        
        # Tab 3: RF Config
        tab3 = ttk.Frame(left, padding="10")
        left.add(tab3, text="RF Config")
        self.build_rf_controls(tab3)
        
        # Tab 4: Pipes & Addresses
        tab4 = ttk.Frame(left, padding="10")
        left.add(tab4, text="Pipes")
        self.build_pipe_controls(tab4)
        
        # Tab 5: Payload
        tab5 = ttk.Frame(left, padding="10")
        left.add(tab5, text="Payload")
        self.build_payload_controls(tab5)
    
    def build_quick_controls(self, parent):
        """Essential quick action buttons"""
        # Mode Control
        mode_frame = ttk.LabelFrame(parent, text="Mode Control", padding="10")
        mode_frame.pack(fill="x", pady=(0, 10))
        
        modes = [
            ("Power Down", "X 0", self.error_color),
            ("Standby", "X 1", self.warning_color),
            ("RX Mode", "X 2", self.success_color),
            ("TX Mode", "X 3", self.accent_color)
        ]
        for i, (name, cmd, color) in enumerate(modes):
            btn = tk.Button(mode_frame, text=name, bg=color, fg="white",
                          command=lambda c=cmd: self.send_command(c), width=12)
            btn.grid(row=0, column=i, padx=5, pady=2)
        
        # System Commands
        sys_frame = ttk.LabelFrame(parent, text="System", padding="10")
        sys_frame.pack(fill="x", pady=(0, 10))
        
        sys_cmds = [
            ("Initialize", "i", self.accent_color),
            ("Reset", "N", self.error_color),
            ("Clear IRQ", "O", self.warning_color),
            ("Dump Regs", "D", self.success_color),
            ("Status", "?", self.accent_color)
        ]
        for i, (name, cmd, color) in enumerate(sys_cmds):
            btn = tk.Button(sys_frame, text=name, bg=color, fg="white",
                          command=lambda c=cmd: self.send_command(c), width=12)
            btn.grid(row=i//3, column=i%3, padx=5, pady=2)
        
        # FIFO Operations
        fifo_frame = ttk.LabelFrame(parent, text="FIFO", padding="10")
        fifo_frame.pack(fill="x", pady=(0, 10))
        
        tk.Button(fifo_frame, text="Flush TX", bg=self.error_color, fg="white",
                 command=lambda: self.send_command("f"), width=12).grid(row=0, column=0, padx=5)
        tk.Button(fifo_frame, text="Flush RX", bg=self.error_color, fg="white",
                 command=lambda: self.send_command("g"), width=12).grid(row=0, column=1, padx=5)
        tk.Button(fifo_frame, text="Reuse TX", bg=self.warning_color, fg="white",
                 command=lambda: self.send_command("u"), width=12).grid(row=0, column=2, padx=5)
        
        # Quick Status
        status_frame = ttk.LabelFrame(parent, text="Quick Status", padding="10")
        status_frame.pack(fill="x", pady=(0, 10))
        
        tk.Button(status_frame, text="Quick Status", bg=self.success_color, fg="white",
                 command=lambda: self.send_command("q"), width=15).pack(pady=2)
        tk.Button(status_frame, text="Carrier Detect", bg=self.accent_color, fg="white",
                 command=lambda: self.send_command("d"), width=15).pack(pady=2)
        
        # Settings
        set_frame = ttk.LabelFrame(parent, text="Settings", padding="10")
        set_frame.pack(fill="x")
        
        tk.Button(set_frame, text="Toggle Echo", bg=self.accent_color, fg="white",
                 command=lambda: self.send_command("."), width=12).grid(row=0, column=0, padx=5)
        tk.Button(set_frame, text="Toggle Verbose", bg=self.accent_color, fg="white",
                 command=lambda: self.send_command(","), width=12).grid(row=0, column=1, padx=5)
    
    def build_register_controls(self, parent):
        """Register read/write interface"""
        # Single Register
        single_frame = ttk.LabelFrame(parent, text="Single Register", padding="10")
        single_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(single_frame, text="Addr (hex):").grid(row=0, column=0, padx=5)
        self.reg_addr = ttk.Entry(single_frame, width=6)
        self.reg_addr.grid(row=0, column=1, padx=5)
        self.reg_addr.insert(0, "00")
        
        ttk.Label(single_frame, text="Data (hex):").grid(row=0, column=2, padx=5)
        self.reg_data = ttk.Entry(single_frame, width=6)
        self.reg_data.grid(row=0, column=3, padx=5)
        self.reg_data.insert(0, "00")
        
        tk.Button(single_frame, text="Read", bg=self.accent_color, fg="white",
                 command=self.read_register, width=8).grid(row=0, column=4, padx=5)
        tk.Button(single_frame, text="Write", bg=self.success_color, fg="white",
                 command=self.write_register, width=8).grid(row=0, column=5, padx=5)
        
        # Multi-byte
        multi_frame = ttk.LabelFrame(parent, text="Multi-Byte", padding="10")
        multi_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(multi_frame, text="Addr:").grid(row=0, column=0, padx=5)
        self.multi_addr = ttk.Entry(multi_frame, width=6)
        self.multi_addr.grid(row=0, column=1, padx=5)
        self.multi_addr.insert(0, "0A")
        
        ttk.Label(multi_frame, text="Data (space-separated hex):").grid(row=1, column=0, columnspan=2, sticky="w", pady=(5,0))
        self.multi_data = ttk.Entry(multi_frame, width=40)
        self.multi_data.grid(row=2, column=0, columnspan=4, sticky="ew", padx=5, pady=5)
        self.multi_data.insert(0, "E7 E7 E7 E7 E7")
        
        btn_frame = ttk.Frame(multi_frame)
        btn_frame.grid(row=3, column=0, columnspan=4, pady=5)
        tk.Button(btn_frame, text="Read Multi", bg=self.accent_color, fg="white",
                 command=self.read_multi, width=12).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Write Multi", bg=self.success_color, fg="white",
                 command=self.write_multi, width=12).pack(side="left", padx=5)
        
        # Register Map Reference
        ref_frame = ttk.LabelFrame(parent, text="Register Map Reference", padding="10")
        ref_frame.pack(fill="both", expand=True, pady=(0, 10))
        
        regs = """00 CONFIG | 01 EN_AA | 02 EN_RXADDR | 03 SETUP_AW
04 SETUP_RETR | 05 RF_CH | 06 RF_SETUP | 07 STATUS
08 OBSERVE_TX | 09 CD/RPD | 0A RX_ADDR_P0 | 0B RX_ADDR_P1
0C RX_ADDR_P2 | 0D RX_ADDR_P3 | 0E RX_ADDR_P4 | 0F RX_ADDR_P5
10 TX_ADDR | 11-16 RX_PW_P0-5 | 17 FIFO_STATUS | 1C DYNPD | 1D FEATURE"""
        
        ref_text = tk.Text(ref_frame, height=8, width=45, font=('Consolas', 9),
                          wrap="word", state="disabled", bg="#f5f5f5")
        ref_text.pack(fill="both", expand=True)
        ref_text.config(state="normal")
        ref_text.insert("1.0", regs)
        ref_text.config(state="disabled")
    
    def build_rf_controls(self, parent):
        """RF channel, power, data rate controls"""
        # Channel
        ch_frame = ttk.LabelFrame(parent, text="RF Channel (0-125)", padding="10")
        ch_frame.pack(fill="x", pady=(0, 10))
        
        self.ch_slider = tk.Scale(ch_frame, from_=0, to=125, orient="horizontal", length=300)
        self.ch_slider.set(2)
        self.ch_slider.pack(fill="x")
        
        btn_frame = ttk.Frame(ch_frame)
        btn_frame.pack(fill="x", pady=5)
        tk.Button(btn_frame, text="Set Channel", bg=self.accent_color, fg="white",
                 command=self.set_channel, width=15).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Get Channel", bg=self.success_color, fg="white",
                 command=lambda: self.send_command("R 05"), width=15).pack(side="left", padx=5)
        
        # TX Power
        pwr_frame = ttk.LabelFrame(parent, text="TX Power", padding="10")
        pwr_frame.pack(fill="x", pady=(0, 10))
        
        self.pwr_var = tk.IntVar(value=3)
        powers = [("0: -18dBm", 0), ("1: -12dBm", 1), ("2: -6dBm", 2), ("3: 0dBm", 3)]
        for text, val in powers:
            ttk.Radiobutton(pwr_frame, text=text, variable=self.pwr_var, value=val).pack(anchor="w")
        
        tk.Button(pwr_frame, text="Apply Power", bg=self.accent_color, fg="white",
                 command=self.set_power, width=15).pack(pady=5)
        
        # Data Rate
        rate_frame = ttk.LabelFrame(parent, text="Data Rate", padding="10")
        rate_frame.pack(fill="x", pady=(0, 10))
        
        self.rate_var = tk.IntVar(value=0)
        rates = [("0: 1 Mbps", 0), ("1: 2 Mbps", 1), ("2: 250 kbps", 2)]
        for text, val in rates:
            ttk.Radiobutton(rate_frame, text=text, variable=self.rate_var, value=val).pack(anchor="w")
        
        tk.Button(rate_frame, text="Apply Rate", bg=self.accent_color, fg="white",
                 command=self.set_rate, width=15).pack(pady=5)
        
        # CRC
        crc_frame = ttk.LabelFrame(parent, text="CRC Mode", padding="10")
        crc_frame.pack(fill="x")
        
        self.crc_var = tk.IntVar(value=2)
        crcs = [("0: Disabled", 0), ("1: 1 Byte", 1), ("2: 2 Bytes", 2)]
        for text, val in crcs:
            ttk.Radiobutton(crc_frame, text=text, variable=self.crc_var, value=val).pack(anchor="w")
        
        tk.Button(crc_frame, text="Apply CRC", bg=self.accent_color, fg="white",
                 command=self.set_crc, width=15).pack(pady=5)
    
    def build_pipe_controls(self, parent):
        """Pipe and address configuration"""
        # Address Width
        aw_frame = ttk.LabelFrame(parent, text="Address Width", padding="10")
        aw_frame.pack(fill="x", pady=(0, 10))
        
        self.aw_var = tk.IntVar(value=5)
        ttk.Spinbox(aw_frame, from_=3, to=5, textvariable=self.aw_var, width=5).pack(side="left", padx=5)
        tk.Button(aw_frame, text="Set", bg=self.accent_color, fg="white",
                 command=self.set_address_width, width=8).pack(side="left", padx=5)
        
        # Pipe Enable/Disable
        pipe_frame = ttk.LabelFrame(parent, text="Pipe Control (0-5)", padding="10")
        pipe_frame.pack(fill="x", pady=(0, 10))
        
        self.pipe_vars = []
        for i in range(6):
            var = tk.BooleanVar(value=(i < 2))
            self.pipe_vars.append(var)
            ttk.Checkbutton(pipe_frame, text=f"Pipe {i}", variable=var).grid(row=i//3, column=i%3, padx=10, pady=2)
        
        btn_frame = ttk.Frame(pipe_frame)
        btn_frame.grid(row=2, column=0, columnspan=3, pady=5)
        tk.Button(btn_frame, text="Apply Pipes", bg=self.accent_color, fg="white",
                 command=self.apply_pipes, width=12).pack(side="left", padx=5)
        
        # TX Address
        tx_addr_frame = ttk.LabelFrame(parent, text="TX Address (hex bytes)", padding="10")
        tx_addr_frame.pack(fill="x", pady=(0, 10))
        
        self.tx_addr = ttk.Entry(tx_addr_frame, width=40)
        self.tx_addr.pack(fill="x", padx=5)
        self.tx_addr.insert(0, "E7 E7 E7 E7 E7")
        
        tk.Button(tx_addr_frame, text="Set TX Address", bg=self.accent_color, fg="white",
                 command=self.set_tx_address, width=15).pack(pady=5)
        
        # RX Pipe 0 Address
        rx_addr_frame = ttk.LabelFrame(parent, text="RX Pipe 0 Address (hex bytes)", padding="10")
        rx_addr_frame.pack(fill="x", pady=(0, 10))
        
        self.rx_addr = ttk.Entry(rx_addr_frame, width=40)
        self.rx_addr.pack(fill="x", padx=5)
        self.rx_addr.insert(0, "E7 E7 E7 E7 E7")
        
        tk.Button(rx_addr_frame, text="Set RX Address", bg=self.accent_color, fg="white",
                 command=self.set_rx_address, width=15).pack(pady=5)
        
        # Auto-ACK
        ack_frame = ttk.LabelFrame(parent, text="Auto-ACK per Pipe", padding="10")
        ack_frame.pack(fill="x", pady=(0, 10))
        
        self.ack_vars = []
        for i in range(6):
            var = tk.BooleanVar(value=True)
            self.ack_vars.append(var)
            ttk.Checkbutton(ack_frame, text=f"Pipe {i}", variable=var).grid(row=i//3, column=i%3, padx=10, pady=2)
        
        btn_frame2 = ttk.Frame(ack_frame)
        btn_frame2.grid(row=2, column=0, columnspan=3, pady=5)
        tk.Button(btn_frame2, text="Apply ACK", bg=self.accent_color, fg="white",
                 command=self.apply_auto_ack, width=12).pack(side="left", padx=5)
        
        # Dynamic Payload
        dpl_frame = ttk.LabelFrame(parent, text="Dynamic Payload per Pipe", padding="10")
        dpl_frame.pack(fill="x", pady=(0, 10))
        
        self.dpl_vars = []
        for i in range(6):
            var = tk.BooleanVar(value=False)
            self.dpl_vars.append(var)
            ttk.Checkbutton(dpl_frame, text=f"Pipe {i}", variable=var).grid(row=i//3, column=i%3, padx=10, pady=2)
        
        btn_frame3 = ttk.Frame(dpl_frame)
        btn_frame3.grid(row=2, column=0, columnspan=3, pady=5)
        tk.Button(btn_frame3, text="Apply DPL", bg=self.accent_color, fg="white",
                 command=self.apply_dynamic_payload, width=12).pack(side="left", padx=5)
        
        # Retransmit
        retr_frame = ttk.LabelFrame(parent, text="Auto-Retransmit", padding="10")
        retr_frame.pack(fill="x")
        
        ttk.Label(retr_frame, text="Delay (0-15):").grid(row=0, column=0, padx=5)
        self.retr_delay = ttk.Spinbox(retr_frame, from_=0, to=15, width=5)
        self.retr_delay.grid(row=0, column=1, padx=5)
        self.retr_delay.set(3)
        
        ttk.Label(retr_frame, text="Count (0-15):").grid(row=0, column=2, padx=5)
        self.retr_count = ttk.Spinbox(retr_frame, from_=0, to=15, width=5)
        self.retr_count.grid(row=0, column=3, padx=5)
        self.retr_count.set(3)
        
        tk.Button(retr_frame, text="Apply", bg=self.accent_color, fg="white",
                 command=self.set_retransmit, width=10).grid(row=0, column=4, padx=10)
    
    def build_payload_controls(self, parent):
        """Payload send/receive"""
        # TX Payload
        tx_frame = ttk.LabelFrame(parent, text="Transmit Payload", padding="10")
        tx_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(tx_frame, text="Data (hex bytes, space separated):").pack(anchor="w")
        self.tx_payload = ttk.Entry(tx_frame, width=50)
        self.tx_payload.pack(fill="x", pady=5)
        self.tx_payload.insert(0, "01 02 03 04 05")
        
        btn_frame = ttk.Frame(tx_frame)
        btn_frame.pack(fill="x")
        tk.Button(btn_frame, text="Send with ACK", bg=self.success_color, fg="white",
                 command=self.send_payload, width=15).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Send NO_ACK", bg=self.warning_color, fg="white",
                 command=self.send_payload_noack, width=15).pack(side="left", padx=5)
        
        # RX Payload
        rx_frame = ttk.LabelFrame(parent, text="Receive Payload", padding="10")
        rx_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(rx_frame, text="Length to read (bytes):").pack(anchor="w")
        self.rx_len = ttk.Spinbox(rx_frame, from_=1, to=32, width=5)
        self.rx_len.pack(anchor="w", pady=5)
        self.rx_len.set(32)
        
        tk.Button(rx_frame, text="Read Payload", bg=self.accent_color, fg="white",
                 command=self.read_payload, width=15).pack(pady=5)
        tk.Button(rx_frame, text="Read Dynamic Width", bg=self.accent_color, fg="white",
                 command=lambda: self.send_command("F"), width=15).pack(pady=5)
        
        # ACK Payload
        ack_pl_frame = ttk.LabelFrame(parent, text="ACK Payload (for PRX mode)", padding="10")
        ack_pl_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(ack_pl_frame, text="Pipe (0-5):").grid(row=0, column=0, padx=5)
        self.ack_pipe = ttk.Spinbox(ack_pl_frame, from_=0, to=5, width=5)
        self.ack_pipe.grid(row=0, column=1, padx=5)
        self.ack_pipe.set(0)
        
        ttk.Label(ack_pl_frame, text="Data (hex):").grid(row=1, column=0, padx=5, pady=5)
        self.ack_payload = ttk.Entry(ack_pl_frame, width=35)
        self.ack_payload.grid(row=1, column=1, columnspan=2, padx=5, pady=5)
        self.ack_payload.insert(0, "AA BB CC DD")
        
        tk.Button(ack_pl_frame, text="Write ACK Payload", bg=self.accent_color, fg="white",
                 command=self.write_ack_payload, width=18).grid(row=2, column=0, columnspan=2, pady=5)
        
        # Payload width per pipe
        pw_frame = ttk.LabelFrame(parent, text="Static Payload Width per Pipe", padding="10")
        pw_frame.pack(fill="x")
        
        self.pw_entries = []
        for i in range(6):
            ttk.Label(pw_frame, text=f"P{i}:").grid(row=i//3, column=(i%3)*2, padx=2)
            entry = ttk.Entry(pw_frame, width=4)
            entry.grid(row=i//3, column=(i%3)*2+1, padx=2)
            entry.insert(0, "0")
            self.pw_entries.append(entry)
        
        tk.Button(pw_frame, text="Apply Widths", bg=self.accent_color, fg="white",
                 command=self.apply_payload_widths, width=15).grid(row=2, column=0, columnspan=6, pady=5)
    
    def build_right_panel(self, parent, row, col):
        """Right side with terminal and advanced controls"""
        right = ttk.Frame(parent)
        right.grid(row=row, column=col, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        
        # Terminal output
        term_frame = ttk.LabelFrame(right, text="Serial Terminal", padding="5")
        term_frame.grid(row=0, column=0, sticky="nsew")
        term_frame.columnconfigure(0, weight=1)
        term_frame.rowconfigure(0, weight=1)
        
        self.terminal = scrolledtext.ScrolledText(term_frame, wrap="word", font=('Consolas', 10),
                                                  state="disabled", bg="#1e1e1e", fg="#d4d4d4",
                                                  insertbackground="white")
        self.terminal.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Terminal controls
        term_ctrl = ttk.Frame(term_frame)
        term_ctrl.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        
        ttk.Label(term_ctrl, text="Raw Command:").pack(side="left", padx=5)
        self.raw_cmd = ttk.Entry(term_ctrl, width=40)
        self.raw_cmd.pack(side="left", padx=5, fill="x", expand=True)
        self.raw_cmd.bind("<Return>", lambda e: self.send_raw_command())
        
        tk.Button(term_ctrl, text="Send", bg=self.accent_color, fg="white",
                 command=self.send_raw_command, width=8).pack(side="left", padx=5)
        tk.Button(term_ctrl, text="Clear", bg=self.error_color, fg="white",
                 command=self.clear_terminal, width=8).pack(side="left", padx=5)
        tk.Button(term_ctrl, text="Save Log", bg=self.success_color, fg="white",
                 command=self.save_log, width=10).pack(side="left", padx=5)
        
        # Advanced controls below terminal
        adv_frame = ttk.LabelFrame(right, text="Advanced / Batch", padding="10")
        adv_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        
        # Batch commands
        ttk.Label(adv_frame, text="Batch Commands (one per line):").pack(anchor="w")
        self.batch_text = tk.Text(adv_frame, height=5, width=60, font=('Consolas', 9))
        self.batch_text.pack(fill="x", pady=5)
        self.batch_text.insert("1.0", "i\nN\nC 02\nX 2\nD")
        
        btn_frame = ttk.Frame(adv_frame)
        btn_frame.pack(fill="x")
        tk.Button(btn_frame, text="Execute Batch", bg=self.warning_color, fg="white",
                 command=self.execute_batch, width=15).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Load Script", bg=self.accent_color, fg="white",
                 command=self.load_script, width=12).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Save Script", bg=self.accent_color, fg="white",
                 command=self.save_script, width=12).pack(side="left", padx=5)
        
        # Quick macros
        macro_frame = ttk.LabelFrame(right, text="Macros", padding="10")
        macro_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        
        macros = [
            ("TX Setup", "i\nN\nC 02\nJ 05\n@ 0 05 E7 E7 E7 E7 E7\nT 3\nB 0\nX 3\nD"),
            ("RX Setup", "i\nN\nC 02\nJ 05\n@ 1 05 E7 E7 E7 E7 E7\nE 0\nL 0 05\nT 3\nB 0\nX 2\nD"),
            ("Dynamic TX", "i\nN\nZ 73\nY 0 1\nC 02\nJ 05\n@ 0 05 E7 E7 E7 E7 E7\nX 3\nD"),
            ("Dynamic RX", "i\nN\nZ 73\nY 0 1\nC 02\nJ 05\n@ 1 05 E7 E7 E7 E7 E7\nE 0\nX 2\nD"),
            ("Scan Channels", "i\nN\nX 2\n" + "\n".join([f"C {c:02X}\nd" for c in range(126)])),
        ]
        
        for i, (name, script) in enumerate(macros):
            tk.Button(macro_frame, text=name, bg=self.accent_color, fg="white",
                     command=lambda s=script: self.load_macro(s), width=15).grid(
                         row=i//3, column=i%3, padx=5, pady=2)
    
    def build_status_bar(self, parent, row, col, colspan):
        """Bottom status bar"""
        self.status_bar = ttk.Label(parent, text="Disconnected", relief="sunken", anchor="w",
                                    font=('Segoe UI', 9))
        self.status_bar.grid(row=row, column=col, columnspan=colspan, sticky="ew", pady=(10, 0))
    
    # ═══════════════════════════════════════════════════════════════════════
    # CONNECTION METHODS
    # ═══════════════════════════════════════════════════════════════════════
    
    def refresh_ports(self):
        """Refresh COM port list"""
        ports = self.controller.list_ports()
        self.port_combo['values'] = [p[1] for p in ports]
        if ports:
            self.port_combo.set(ports[0][1])
            self.port_var.set(ports[0][1])
    
    def get_selected_port(self):
        """Extract COM port name from dropdown"""
        selected = self.port_var.get()
        match = re.match(r'(COM\d+)', selected)
        if match:
            return match.group(1)
        return selected.split()[0] if selected else None
    
    def toggle_connection(self):
        """Connect or disconnect"""
        if self.controller.connected:
            self.disconnect()
        else:
            port = self.get_selected_port()
            if not port:
                messagebox.showerror("Error", "Please select a COM port")
                return
            
            baud = int(self.baud_var.get())
            result = self.controller.connect(port, baud)
            
            if result is True:
                self.connected_ui_state(True)
                self.log(f"Connected to {port} at {baud} baud\n", "success")
                time.sleep(2)
                self.controller.clear_buffer()
            else:
                messagebox.showerror("Connection Failed", str(result))
    
    def disconnect(self):
        """Disconnect from serial"""
        self.controller.disconnect()
        self.connected_ui_state(False)
        self.log("Disconnected\n", "warning")
    
    def connected_ui_state(self, connected):
        """Update UI based on connection state"""
        self.connect_btn.config(text="Disconnect" if connected else "Connect")
        self.disconnect_btn.config(state="normal" if connected else "disabled")
        self.port_combo.config(state="disabled" if connected else "readonly")
        
        color = self.success_color if connected else self.error_color
        self.status_canvas.itemconfig(self.status_indicator, fill=color)
        self.status_bar.config(text=f"{'Connected' if connected else 'Disconnected'} - {self.get_selected_port() or 'None'}")
    
    # ═══════════════════════════════════════════════════════════════════════
    # COMMAND METHODS
    # ═══════════════════════════════════════════════════════════════════════
    
    def send_command(self, cmd, wait=False):
        """Send command to Arduino"""
        if not self.controller.connected:
            self.log("Not connected!\n", "error")
            return
        
        self.log(f">>> {cmd}\n", "command")
        response = self.controller.send(cmd, wait_for_response=wait, timeout=3)
        if wait and response and response != "Timeout":
            self.log(f"<<< {response}\n", "response")
        return response
    
    def send_raw_command(self):
        """Send raw command from entry box"""
        cmd = self.raw_cmd.get().strip()
        if cmd:
            self.send_command(cmd, wait=True)
            self.raw_cmd.delete(0, tk.END)
    
    def read_register(self):
        addr = self.reg_addr.get().strip()
        if addr:
            self.send_command(f"R {addr}", wait=True)
    
    def write_register(self):
        addr = self.reg_addr.get().strip()
        data = self.reg_data.get().strip()
        if addr and data:
            self.send_command(f"W {addr} {data}", wait=True)
    
    def read_multi(self):
        addr = self.multi_addr.get().strip()
        data_str = self.multi_data.get().strip()
        count = len(data_str.split()) if data_str else 5
        self.send_command(f"n {addr} {count:02X}", wait=True)
    
    def write_multi(self):
        addr = self.multi_addr.get().strip()
        data = self.multi_data.get().strip()
        if addr and data:
            count = len(data.split())
            self.send_command(f"M {addr} {count:02X} {data}", wait=True)
    
    def set_channel(self):
        self.send_command(f"C {self.ch_slider.get():02X}")
    
    def set_power(self):
        self.send_command(f"T {self.pwr_var.get()}")
    
    def set_rate(self):
        self.send_command(f"B {self.rate_var.get()}")
    
    def set_crc(self):
        self.send_command(f"r {self.crc_var.get()}")
    
    def set_address_width(self):
        self.send_command(f"J {self.aw_var.get()}")
    
    def apply_pipes(self):
        for i, var in enumerate(self.pipe_vars):
            self.send_command(f"{'E' if var.get() else 'K'} {i}")
    
    def set_tx_address(self):
        addr = self.tx_addr.get().strip()
        count = len(addr.split())
        self.send_command(f"@ 0 {count:02X} {addr}")
    
    def set_rx_address(self):
        addr = self.rx_addr.get().strip()
        count = len(addr.split())
        self.send_command(f"@ 1 {count:02X} {addr}")
    
    def apply_auto_ack(self):
        for i, var in enumerate(self.ack_vars):
            self.send_command(f"U {i} {1 if var.get() else 0}")
    
    def apply_dynamic_payload(self):
        for i, var in enumerate(self.dpl_vars):
            self.send_command(f"Y {i} {1 if var.get() else 0}")
    
    def set_retransmit(self):
        delay = int(self.retr_delay.get())
        count = int(self.retr_count.get())
        self.send_command(f"V {delay:02X} {count:02X}")
    
    def send_payload(self):
        data = self.tx_payload.get().strip()
        count = len(data.split())
        self.send_command(f"P {count:02X} {data}")
    
    def send_payload_noack(self):
        data = self.tx_payload.get().strip()
        count = len(data.split())
        self.send_command(f"p {count:02X} {data}")
    
    def read_payload(self):
        length = int(self.rx_len.get())
        self.send_command(f"G {length:02X}", wait=True)
    
    def write_ack_payload(self):
        pipe = int(self.ack_pipe.get())
        data = self.ack_payload.get().strip()
        count = len(data.split())
        self.send_command(f"A {pipe} {count:02X} {data}")
    
    def apply_payload_widths(self):
        for i, entry in enumerate(self.pw_entries):
            width = entry.get().strip()
            if width:
                self.send_command(f"L {i} {int(width):02X}")
    
    def execute_batch(self):
        script = self.batch_text.get("1.0", tk.END).strip()
        if not script:
            return
        
        def run():
            for line in script.split('\n'):
                line = line.strip()
                if line and not line.startswith('#'):
                    self.root.after(0, lambda l=line: self.send_command(l, wait=True))
                    time.sleep(0.3)
        
        threading.Thread(target=run, daemon=True).start()
    
    def load_macro(self, script):
        self.batch_text.delete("1.0", tk.END)
        self.batch_text.insert("1.0", script)
    
    def load_script(self):
        path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if path:
            with open(path, 'r') as f:
                self.batch_text.delete("1.0", tk.END)
                self.batch_text.insert("1.0", f.read())
    
    def save_script(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                                           filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if path:
            with open(path, 'w') as f:
                f.write(self.batch_text.get("1.0", tk.END))
    
    def save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                                           filetypes=[("Text files", "*.txt"), ("Log files", "*.log")])
        if path:
            with open(path, 'w') as f:
                f.write(self.terminal.get("1.0", tk.END))
    
    # ═══════════════════════════════════════════════════════════════════════
    # TERMINAL & UI UPDATES
    # ═══════════════════════════════════════════════════════════════════════
    
    def on_serial_data(self, data):
        self.pending_data += data
    
    def update_queue(self):
        if self.pending_data:
            self.log(self.pending_data, "incoming")
            self.pending_data = ""
        self.root.after(100, self.update_queue)
    
    def log(self, text, tag="normal"):
        colors = {
            "normal": "#d4d4d4",
            "command": "#569cd6",
            "response": "#4ec9b0",
            "success": "#b5cea8",
            "error": "#f44747",
            "warning": "#ce9178",
            "incoming": "#c586c0",
        }
        
        self.terminal.config(state="normal")
        self.terminal.insert(tk.END, text, tag)
        self.terminal.tag_config(tag, foreground=colors.get(tag, colors["normal"]))
        self.terminal.see(tk.END)
        self.terminal.config(state="disabled")
    
    def clear_terminal(self):
        self.terminal.config(state="normal")
        self.terminal.delete("1.0", tk.END)
        self.terminal.config(state="disabled")


def main():
    root = tk.Tk()
    app = NRF24GUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()