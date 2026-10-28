# NOHRD Ciclotte Controller — Projektkontext

Zephyr/nRF Connect SDK Firmware (nRF52832, D52Q-Modul / Dev-Kit-Breadboard)
für den NOHRD-Ciclotte-Displaycontroller. Abgeleitet aus `nohrd_bike`
(Git-Historie bis `d1d3898` ist die Bike-Historie). Entspricht in der alten
AVR-Firmware der Konfiguration `#define BIKE` + `#define CICLOTTE`.

Treibt das serielle Display, liest die Trittfrequenz, stellt die Bremse über
einen **Servo per PWM** (Stufe 1–10 aus der GUI), exponiert BLE FTMS
(Indoor Bike Data) und relayt Daten von BLE-Herzfrequenzgurten.

## Wie die AVR-Firmware auf den D52Q portiert wurde (Analyse 2026-10-28)

`src/atmegamain.c` lief auf einem ATmega für drei Geräte (`SPRINTBOK`,
`BIKE`, `BIKE`+`CICLOTTE`) und sprach per UART mit einem externen GFIT-Modul.
Im D52Q-Port (`nohrd_bike`, Konfiguration nur `BIKE`):

- GFIT-Modul entfällt: BLE läuft nativ auf dem nRF52832 (FTMS, Custom-NOHRD-
  Service, HR-Gurt-Relay). Der GFIT-Modellstring (`gf13`: NOHRDNB/NOHRDNC)
  ist jetzt `CONFIG_BT_DEVICE_NAME`.
- `App_State`-Zustandsautomat → Superloop mit Seitentabelle `pages[]` und
  `run_button_action()`. Display-Protokoll unverändert.
- ADC/EEPROM → Zephyr-ADC (AIN1) + NVS im internen Flash.
- Trittfrequenz per GPIO-Interrupt (P0.11).
- Neu: OTA über MCUboot, Secondary-Slot auf externem SPI-NOR.
- Leistungs-/Geschwindigkeitsformeln 1:1 übernommen.
- AVR-CICLOTTE-Vorlage: `setServoAngle()`, `EnableDisablePWM()`,
  `BTN_ACTION_INCREASE/DECREASE_ANGLE`, `actResUpdate()` (`ResistanceNom =
  (idx+1)/10`), Seitenlayouts `page_Session`/`page_bt_connection`.

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

## Gefundene Fehler (in beiden Projekten behoben)

- DFU-Build linkte nicht: `CONFIG_BT_GATT_DYNAMIC_DB=y` fehlte in
  `sysbuild/<image>/prj.conf` (dynamischer HR-Service).
- `heart_rate_attrs` in `ble_ftms.c` war `const`, aber
  `bt_gatt_service_register()` schreibt Handles hinein → jetzt nicht-const.
- In `nohrd_bike` als Commit `521918a` nachgezogen und nach GitHub
  (Waterrower/BikeD52) gepusht.

## Repo / Git

- Eigenes lokales Repo, **kein Remote** (Bike-`origin` wurde entfernt).
  Für GitHub ein Repo anlegen (analog `Waterrower/Sprintbok_D52`), dann
  `git remote add origin <url>` + `git push -u origin master`.
- Ableitungs-Commit: `3e0eacd` („Derive Ciclotte controller from nohrd_bike").

## Build-Umgebung (für Builds außerhalb von VS Code)

`tools/build-profile.ps1` braucht die NCS-Toolchain im PATH
(`C:\ncs\toolchains\936afb6332`: `opt\bin`, `opt\zephyr-sdk\arm-zephyr-eabi\bin`,
`ZEPHYR_TOOLCHAIN_VARIANT=zephyr`, `ZEPHYR_SDK_INSTALL_DIR`,
`ZEPHYR_BASE=C:\ncs\v3.3.1\zephyr`). In PowerShell 5.1 die Ausgabe nicht mit
`*>`/`2>&1` umleiten — das Skript läuft mit `$ErrorActionPreference=Stop`,
und CMake-Ausgaben auf stderr brechen den Build dann ab.
Stand 2026-10-28: App- und DFU-Build grün, am echten Gerät noch nicht getestet.

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
