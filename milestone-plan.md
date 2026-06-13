### Milestone-Plan: CANopen Node Discovery → Identify → Device Pages (AP04 + Dunker + Standard)

#### Zielbild (End-to-End Ablauf)
1. **Tools → Ping Scan (1..32)** auf *aktueller* Baudrate (kein Baudwechsel)
2. **Found Nodes Liste** wird gefüllt (Node-ID + Status)
3. **Identify All**: liest pro gefundenem Node die CANopen-Identity (0x1018)
4. **Connect** (pro Node oder Selected): startet die Master-Connect-Pipeline
5. **Device Router** öffnet automatisch:
   - **AP04 Page** (existiert bereits)
   - **Dunker Page** (neu, DS402)
   - **Standard CANopen Page** (für unbekannte Geräte)

---

## Milestone 0 — Cleanup / Voraussetzungen (0.5–1 Tag)
**Ziel:** Basis konsistent machen, damit die nächsten Schritte nicht in Refactors stecken bleiben.

**Deliverables**
- Nur noch **eine** Start-Menü-Quelle: `src/ui/start_menu_ui.h` (mit 2 Screens: Start + Tools)
- Sketch-Includes im gewünschten Stil:
  - `#include "src/ui/start_menu_ui.h"`
  - `#include "src/canopen/canopen_driver.h"`
- Entfernen/abschalten von unbenutzten Variablen (z.B. dunkerForceBaud/dunkerBaud), wenn Feature nicht aktiv ist.

**Akzeptanzkriterien**
- Kompiliert ohne Fehler, ohne neue Warnings (außer bewusst tolerierte).
- Startscreen + Tools-Screen funktionieren, nichts off-screen.

**Risiken**
- Lokale Projektstruktur kann von Tooling-Pfaden abweichen → Änderungen als Copy/Paste Blöcke liefern.

---

## Milestone 1 — Tools Screen: Found Nodes Liste + Selector + Connect (1–2 Tage)
**Ziel:** Deine gewünschte Bedienlogik “Scan → Liste → Selected Node → Connect” ist vollständig vorhanden.

**Deliverables**
- Tools-Screen:
  - Button: **PING SCAN 1..32**
  - **Found Nodes List** (LVGL list)
    - Zeile: `Node X` + “SELECT”/Row-Click
    - **CONNECT** Button pro Node (direkt)
  - Anzeige: **Selected: X**
  - Button: **CONNECT Selected**
- Backend:
  - Ping Scan sammelt *alle* Antworten (nicht beim ersten Stopp)
  - UI wird nach Scan aktualisiert (Liste enthält alle gefundenen Nodes)
  - `CONNECT` triggert `master.connectNode(nodeId, KnownDeviceType::Unknown)`

**Akzeptanzkriterien**
- Ping Scan @125k findet mehrere Nodes (wenn vorhanden) und listet sie.
- Node auswählen klappt zuverlässig.
- Connect pro Node / Selected führt deterministisch zu Master-Connect (Serial Logs nachvollziehbar).

---

## Milestone 2 — Identify All + Klassifizierung (1–2 Tage)
**Ziel:** Nach dem Scan werden die Nodes “wer/was” und die UI kann passend routen.

**Deliverables**
- Button: **IDENTIFY ALL**
  - liest pro Found Node:
    - 0x1018:01 Vendor ID
    - 0x1018:02 Product Code
    - 0x1018:03 Revision
    - 0x1018:04 Serial
  - Ergebnis wird:
    - im Serial ausgegeben
    - in der Found Nodes List als Kurzinfo angezeigt (z.B. Vendor/Product)
- Klassifizierung/Mapping:
  - (Vendor, Product) → KnownDeviceType
  - Unbekannt → KnownDeviceType::Unknown

**Akzeptanzkriterien**
- Identify läuft non-blocking (UI bleibt bedienbar).
- Bei bekannten Geräten wird der Typ korrekt angezeigt (AP04, Dunker).

**Offene Inputs**
- Dunker Vendor/Product IDs (aus 0x1018) müssen einmal sauber erfasst werden.

---

## Milestone 3 — Standard CANopen Page (Unknown Devices) (1–2 Tage)
**Ziel:** Für unbekannte Nodes gibt es eine “Standard Seite”, die immer funktioniert.

**Deliverables**
- Standard Page Inhalte:
  - NMT: Start / Pre-Op / Stop / Reset Node / Reset Comm
  - Anzeige Identity (0x1018) falls bekannt
  - Anzeige Error register (0x1001), Error code (0x603F falls DS402), Statusword (0x6041 falls DS402)
  - Optional: Heartbeat State (wenn empfangen)

**Akzeptanzkriterien**
- Jede gefundene Node kann “connected” werden und öffnet mindestens Standard Page.

---

## Milestone 4 — Dunker Page v1 (DS402 Basis) (2–4 Tage)
**Ziel:** Stabiler DS402 Grundbetrieb: Status/Fault/Enable.

**Deliverables**
- Dunker Page v1:
  - Live: Statusword 0x6041, Error code 0x603F, Error register 0x1001
  - Buttons:
    - Fault Reset (0x6040=0x0080)
    - Shutdown (0x6040=0x0006)
    - Switch On (0x6040=0x0007)
    - Enable Operation (0x6040=0x000F)
  - Optional: Anzeige zuletzt empfangener EMCY + Code

**Akzeptanzkriterien**
- Drive geht aus Fault/Disabled in “Operation enabled” (Statusword bestätigt), sofern Hardware es zulässt.

---

## Milestone 5 — Dunker Page v2 (Position & Jog) (3–6 Tage)
**Ziel:** Verfahrfunktionen und Zielposition setzen.

**Deliverables**
- Mode of Operation (0x6060) set/read
- Target Position (0x607A) set/read
- Controlword Bit-Handling für “New set-point” + “Change immediately”
- UI:
  - Zielposition Eingabe
  - “Move” Button
  - “Jog + / Jog -” (Hand verfahren)
  - “Override max 10%” (Software-Limit/Clamp)

**Akzeptanzkriterien**
- Positioning funktioniert reproduzierbar (Statusword/Actual position ggf. über 0x6064).

**Offene Inputs**
- Welche Units (counts, mm, deg) und Skalierung gewünscht.

---

## Milestone 6 — Dunker Page v3 (I/O, Brake, Advanced) (variabel, 4–10 Tage)
**Ziel:** Bremse, Ein-/Ausgänge, herstellerspezifische Features.

**Deliverables (abhängig von OD/EDS)**
- Inputs lesen / Outputs setzen
- Brake release / brake control
- Drive reset / quickstop behaviors

**Akzeptanzkriterien**
- Alle Funktionen sind über dokumentierte Objekte implementiert (kein Raten).

**Benötigt**
- Dunker EDS / Objektverzeichnis oder klare Indizes/Subindizes.

---

### Empfohlene Teststrategie (durchgehend)
- Immer: `PING SCAN` → `IDENTIFY ALL` → `CONNECT`
- Für jede neue Funktion: Serial Log + Sniffer (SDO Req/Resp IDs 0x60x/0x58x)
- Bei Busproblemen: Loopback(NO_ACK) als Diagnose (TX ohne ACK)

### Definition of Done (Projekt)
- Mehrere Dunker am Bus werden erkannt, gelistet, identifiziert.
- User kann einen Node gezielt auswählen und bedienen.
- AP04 und Dunker haben eigene Seiten; Unbekannte haben Standardseite.
