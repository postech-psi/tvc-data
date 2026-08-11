# Raspberry Pi 5 — Network Access Reference

Reference for connecting to the Raspberry Pi 5 companion computer from a Windows or macOS client.

## 1. Device

| Property | Value |
|----------|-------|
| Hostname | `2026ugrp3` |
| Username | `ugrp` |
| Password | `2026ugrp3` |
| OS | Raspberry Pi OS (Debian 13) |

Credentials are shared defaults and should be changed before deployment on a shared network (see Section 7).

## 2. Connection Methods

| Method | Pi address | Address stability | Internet on client |
|--------|-----------|-------------------|--------------------|
| Ethernet (direct cable) | `192.168.2.2` | Fixed | No |
| Wi-Fi (`postech`) | Assigned by DHCP | Variable | Yes |
| Onboard hotspot (`ugrp3`) | `10.42.0.1` | Fixed | No |

Use Ethernet or the onboard hotspot when a fixed, predictable address is required (field operation, demonstrations). Use Wi-Fi when the client or the Pi requires internet access (e.g. `git`, `apt`).

## 3. Ethernet (Direct Cable)

The Pi uses a fixed address of `192.168.2.2`. Configure the client's Ethernet interface with a static address on the same subnet before connecting.

**Windows** — Settings → Network & Internet → Ethernet → IP assignment → Edit → Manual → IPv4 on:
- IP address: `192.168.2.1`
- Subnet mask: `255.255.255.0`
- Gateway: leave blank

**macOS** — System Settings → Network → [USB Ethernet adapter] → Details → TCP/IP:
- Configure IPv4: Manually
- IP address: `192.168.2.1`
- Subnet mask: `255.255.255.0`
- Router: leave blank

Connect using the address `192.168.2.2` (Section 6).

## 4. Wi-Fi (Shared Network)

The Pi obtains its address from DHCP, so the address must be determined at each session. The `.local` hostname is unavailable on networks that block mDNS (most campus and enterprise Wi-Fi).

Determine the current address by either method:
- Query the Pi directly (via cable or hotspot): `ip -brief addr show wlan0`
- Scan from the client — the Pi's MAC address begins `2C:CF:67`:
  - Windows: run `arp -a`, then locate the entry beginning `2c-cf-67`
  - macOS: run `arp -a | grep 2c:cf:67`

Both devices must be on the same network.

## 5. Onboard Hotspot

The Pi can broadcast its own Wi-Fi network, `ugrp3`, with a fixed gateway address of `10.42.0.1`. No external router is required.

Control (run on the Pi):
- Enable hotspot: `sudo nmcli con up ugrp3`
- Return to Wi-Fi: `sudo nmcli con up postech`

The hotspot provides a local link to the Pi only and does not provide internet access: the single Wi-Fi radio cannot host the network and connect to an upstream Wi-Fi network at the same time.

Join `ugrp3` from the client, then connect using the address `10.42.0.1`.

## 6. Access Methods

### SSH (command line)
- Windows: PowerShell → `ssh ugrp@<address>`
- macOS: Terminal → `ssh ugrp@<address>`

Accept the host key on first connection, then authenticate with the password.

### Remote Desktop / RDP (graphical)
- Windows: Remote Desktop Connection (`mstsc`) → enter `<address>` → accept the certificate warning.
  Resolution: Show Options → Display → resolution slider (set before connecting).
- macOS: Windows App (Microsoft) → add PC `<address>`.
  Resolution: edit PC → Display → select a resolution.

At the login screen, select session type `Xorg` and authenticate as `ugrp`.

## 7. Security Notes

- The default password matches the hostname. Change it with `passwd`.
- The `ugrp3` hotspot is unsecured by default. Assign a WPA password before use on shared premises:
  `sudo nmcli con modify ugrp3 wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<password>"`

---
*Last updated: 2026-07-24*