# ⚡ BLDC Arduino Controller

Et simpelt men avancerbart projekt til at styre en **BLDC-motor** via en **Arduino UNO** og en ekstern **BLDC-controller** (RS232/PWM-baseret).  
Systemet giver mulighed for **soft-start, hastighedsstyring, retningsskift** og **serielforbindelse** til PC for manuel kontrol.

---

## 🧠 Funktioner

- Automatisk **soft-start** og **soft-stop** for glidende acceleration
- **PWM-styring** af hastighed (0–100 %)
- **Retningsskift (CW/CCW)** via digitalt signal
- **Seriel styring** fra PC:
  - `s` → Start  
  - `x` → Stop  
  - `+ / -` → Juster hastighed ±5 %  
  - `0–9` → Direkte hastighed (0 %=0, 9 %=90 %)  
  - `d` → Skift retning  
  - `r` → EN-reset  
  - `?` → Print status
- Mulighed for **joystick-input** (planlagt)
- Kører **uafhængigt af PC**, men kan kontrolleres via USB-terminal (f.eks. Serial Monitor)

---

## ⚙️ Hardware-opsætning

| Arduino Pin | Controller Pin | Funktion            |
|--------------|----------------|---------------------|
| D9           | SV (PWM)       | Hastigheds-signal   |
| D8           | EN             | Enable-signal (HIGH = aktiv) |
| D7           | FR             | Retning (LOW = CW, HIGH = CCW) |
| GND          | GND            | Fælles jord         |

> ⚠️ **Vigtigt:** Fælles GND mellem Arduino og controller er nødvendigt!

Forsyning: 24 V / 48 V BLDC-controller (ikke fra Arduino-5 V)

---

## 🔌 Software

**Platform:** Arduino UNO / Nano  
**Sprog:** C++ (Arduino-framework)  
**IDE:** Arduino IDE eller VS Code + PlatformIO  

**Filstruktur:**
Motorstyring/
├── Motorstyring.ino # Hovedkode
├── Manual.xlsx # Pin- og funktionsoversigt
└── README.md # Dokumentation

---

## 🧩 Krævede biblioteker

Ingen eksterne biblioteker – kun standard Arduino-funktioner.

---

## 💻 Brug fra PC (Serial Monitor)

1. Upload `Motorstyring.ino` til din Arduino UNO  
2. Åbn Serial Monitor (`Ctrl+Shift+M`)  
3. Vælg:
   - **Baudrate:** `115200`
   - **Line ending:** `No line ending`
4. Brug kommandoerne (`s`, `x`, `+`, `d`, osv.) for at styre motoren.

---

## 🧱 Fremtidige udvidelser

- 🕹️ Joystick-modul (analog input for hastighed og retning)
- 📡 Trådløs modtager (f.eks. RC receiver)
- 🧠 PID-styring for jævn motorrespons
- 🌐 Web-interface via ESP32 eller Wi-Fi-modul

---

## 🔧 Fejlfinding

| Problem | Løsning |
|----------|----------|
| Controller blinker | Tæl blink (se manual) – typisk manglende Hall-signal eller EN = LOW |
| Motor kører ikke | Tjek GND, EN, PWM-signal, og forsyning |
| Alarm “2 blink” | Hall V fejl → kontroller HB-ledning fra motor |
| “Permission denied” ved `git push` | Sørg for aktiv SSH-agent (`ssh-add`) |

---

## 🧑‍💻 Forfatter

**Jean Johnsen**  
GitHub: [@Jeanjohnsen](https://github.com/Jeanjohnsen)  
Mail: [jeanjohnsen@pm.me](mailto:jeanjohnsen@pm.me)

---

## 📜 Licens

Dette projekt distribueres under **MIT License** – du må frit bruge, ændre og distribuere koden, så længe du beholder denne licensfil.
