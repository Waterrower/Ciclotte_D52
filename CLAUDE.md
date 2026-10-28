# NOHRD Ciclotte Controller — Projektkontext

Zephyr/nRF Connect SDK Firmware (nRF52832, D52Q-Modul / Dev-Kit-Breadboard)
für den NOHRD-Ciclotte-Displaycontroller. Abgeleitet aus `nohrd_bike`
(Git-Historie bis `d1d3898` ist die Bike-Historie). Entspricht in der alten
AVR-Firmware der Konfiguration `#define BIKE` + `#define CICLOTTE`.

Treibt das serielle Display, liest die Trittfrequenz, stellt die Bremse über
einen **Servo per PWM** (Stufe 1–10 aus der GUI), exponiert BLE FTMS
(Indoor Bike Data) und relayt Daten von BLE-Herzfrequenzgurten.

## Ciclotte vs. Bike — Unterschiede

Variante wird in `src/main.c` oben per `#define BIKE` / `#define CICLOTTE`
gewählt (wie in `atmegamain.c`). Alle Abweichungen stehen in
`#ifdef CICLOTTE`-Blöcken, der `#else`-Zweig ist unverändert der Bike-Code.
Achtung: Die Overlays dieses Projekts haben keinen ADC mehr — ein Build
ohne `CICLOTTE` braucht wieder die ADC-Overlays aus `nohrd_bike`.

- **Widerstand**: kein Poti/ADC, keine Kalibrierung. `servo_level` (0..9)
  wird über die +/−-Buttons gesetzt, Winkeltabelle
  `{53,67,82,97,110,115,123,130,150,180}°` aus der AVR-Firmware.
  Pulsbreite 0,5–2,5 ms (0–180°), Periode 20 ms.
- **Widerstand intern** = `(level+1)*10` % → Leistungsformel und FTMS
  (×2 = 0,5 %-Auflösung) wie beim Bike. **Anzeige** = Stufe 1–10.
- **Session-Seite**: Tile 4 oben/unten = +/− (statt Pause/Stop),
  **Stop = Tippen auf Tile 0 (Kopfzeile)**, keine Pause — wie AVR.
- **BT-Verbindungsseite**: Tile 0 = Session beenden, Tile 4 = +/−.
- **PWM an/aus** wie AVR `EnableDisablePWM`: an beim Boot, Session-Start,
  BT-Session-Start; aus bei Session-Ende (Summary), BT-Trennung und
  Display-Aus (entspricht AVR-Deep-Sleep). Aus = Puls 0 → Servo hält nicht.
- **Settings 2**: Kalibrier-Kachel und Kalibrier-Reset (Tile 0) entfallen.
- **Custom NOHRD BLE Service**: meldet immer „kalibriert", Kalibrier-Writes
  werden abgelehnt.
- **BLE-Name** `NOHRDNC` (AVR/GFIT-Modellstring Ciclotte; Bike = `NOHRDNB`).
- **Developer-Seite** zeigt Servo-Stufe/Winkel/Ausgang statt ADC-Rohwert.

## Hardware

- Servo-PWM auf **P0.04** (PWM0 Kanal 0, D52Q-M6 J102.05). Board-Default
  für PWM0 wäre P0.17 = grüne LED, daher in allen Overlays umgelegt.
- P0.25–P0.30 meiden: beim D52Q-M6-Devmodul am Beschleunigungssensor.
- Details: [HARDWARE_PROFILES.md](HARDWARE_PROFILES.md).

## Build & Flashen

- `tools/build-profile.ps1 -Target app -Pristine` → `build_ciclotte_app`
- `tools/build-profile.ps1 -Target dfu -Pristine` → `build_ciclotte_dfu`
  (MCUboot + signiertes Image + `dfu_application.zip`).
- **Sysbuild benennt das App-Image nach dem Projektordner** (`Ciclotte`).
  Deshalb heißt das Image-Config-Verzeichnis `sysbuild/Ciclotte/`, und das
  Build-Skript leitet `-D<image>_DTC_OVERLAY_FILE` aus dem Ordnernamen ab.
  Wird der Ordner umbenannt, muss `sysbuild/<neuer Name>/` mitgezogen werden
  (das Skript bricht sonst mit einer Meldung ab).
- `sysbuild/Ciclotte/prj.conf` **ersetzt** im DFU-Build die `prj.conf` —
  Kconfig-Änderungen immer in beiden Dateien pflegen.
- VS Code: Run-and-Debug → „Launch Ciclotte/build_ciclotte_app".

## Repo-Struktur

- `src/main.c`, `src/ble_ftms.c/h`, `src/ble_hr.c/h` — einziger aktiv
  kompilierter Code. Superloop, kein RTOS-Threading.
- `src/atmegamain.c`, `src/addStrings.c`, `parsedisplay.c`, `sample_usart.c` —
  Referenzcode der alten AVR-Firmware (enthält die CICLOTTE-Vorlage:
  `setServoAngle`, `EnableDisablePWM`), NICHT im Build, nicht löschen.
- `d52q_gfit_bridge/`, `nohrd_legacy_gfit_app/`, `diagnostics/` — aus dem
  Bike-Projekt übernommene Referenzen/Bring-up-Harnesses.
- `keys/nohrd-ciclotte-mcuboot-ec256.pem` — **eigener privater
  MCUboot-Signierschlüssel** (neu erzeugt, nicht der Bike-Schlüssel →
  Bike-Images lassen sich nicht per OTA auf eine Ciclotte spielen und
  umgekehrt). Gitignored — niemals `git add -f`. Separat sichern!

## Gegenüber `nohrd_bike` mitgefixt

- DFU-Build linkte nicht: `CONFIG_BT_GATT_DYNAMIC_DB=y` fehlte in
  `sysbuild/<image>/prj.conf` (dynamischer HR-Service).
- `heart_rate_attrs` in `ble_ftms.c` war `const`, aber
  `bt_gatt_service_register()` schreibt Handles hinein → jetzt nicht-const.

## Offene Punkte

- Servo am echten Gerät prüfen: Drehrichtung/Endlagen der Winkeltabelle,
  3,3-V-Signalpegel, separate 5-V-Versorgung.
- Bremse hält ohne PWM nicht (Servo stromlos nach Session-Ende) — so wie
  bei AVR; prüfen, ob die Bremse dabei wegdriftet.
- FTMS-Control-Point („Set Target Resistance") ist nicht implementiert —
  Apps können die Bremse nicht stellen, obwohl die Ciclotte motorisch ist.
- Servo-Stufe wird nicht in NVS gespeichert (wie AVR: nach Reset Stufe 1).
- Aus `nohrd_bike` geerbt: magische Zahlen (`FLYWHEEL_TO_PEDAL_RATIO_X100`,
  Power-/Speed-Koeffizienten), VLA in `display_cmd()`, HR-Register/Unregister
  synchron im BLE-Callback-Kontext. Sonderzeichen in Text-Tabellen und
  `resistance_percent * 2` in FTMS bewusst so.
