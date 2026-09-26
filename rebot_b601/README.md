# rebot_b601: IK, skrypty ruchu i serwer MCP dla reBot Arm B601-RS

> **Scope note for the `sorter` project:** this folder is standalone code for the team's arm, the **reBot Arm B601-RS (RobStride motors)**
> ([D-011](../docs/decisions.md)). It talks to the motors through `motorbridge` directly (not through `reBotArm_control_py`) and can seed block 5's
> `ArmDriver`: kinematics, trajectories, safety checks, simulator. The folder is excluded from the repository-wide `ruff` run (see `ruff.toml`)
> and from the root `pytest` (`testpaths = ["tests"]`).
>
> The sorter uses it as a path dependency ([D-014](../docs/decisions.md)): `sorter.arm` plans with `rebot_b601.arm.plan_path`
> (module-level, also `check_path` / `check_limits`) and drives the arm with `Arm.execute_path`, `Arm.set_gripper`, `Arm.joints`.
> In `pyproject.toml` only `numpy` is required; `motorbridge` is the `hardware` extra and `mcp` the `mcp` extra
> (the standalone install below still uses `requirements.txt`).

Kinematyka (FK/IK), skrypty „jedź do punktu xyz” oraz serwer MCP, dzięki któremu agent może sam poruszać ramieniem.

> **Ważne: kod sprzętowy nie był uruchomiony na prawdziwym ramieniu.**
> Kinematyka i logika sterowania są przetestowane (29 testów, symulator), ale backend
> `HardwareBackend` napisano na podstawie referencyjnego kodu Seeed i nie był jeszcze
> sprawdzony na Twoim ramieniu. Serwer MCP nie był ani uruchamiany, ani importowany.
> Zacznij od symulatora, potem tryb tylko-do-odczytu, potem bardzo małe ruchy
> z ręką przy wyłączniku zasilania.

## Skąd geometria

URDF `00-arm-rs_asm-v3` z oficjalnego repo Seeed
(`Seeed-Projects/lerobot-robot-seeed-b601`, licencja MIT; ten sam łańcuch ramienia jest w
`Seeed-Projects/reBotArm_control_py`). W oficjalnym SDK kąt silnika = kąt przegubu URDF,
więc po kalibracji zera `q` z FK/IK to wprost pozycja silnika. Silniki: 1–3 to `rs-06`,
4–7 to `rs-00`; wzmocnienia POS_VEL z `rebotarm_rs.yaml` SDK.

IK jest własną implementacją (Levenberg–Marquardt, limity przegubów, multi-start, numpy),
bez zależności od Pinocchio. Dokładność < 1 mm, ok. 2–5 ms na rozwiązanie.

## Konwencje

* Układ bazy: **+X do przodu, +Y w lewo, +Z w górę**, początek na płycie bazy (oś przegubu 1 na z = 0,075 m). Jednostki: metry i stopnie.
* TCP = ramka `gripper_end` (koniec chwytaka). Oś natarcia (*approach*) chwytaka to jego +X, od nadgarstka do palców.
* Poza domowa (wszystkie przeguby 0°): złożone ramię, TCP ok. `(0,30; 0,00; 0,22)` m, chwytak celuje poziomo do przodu.
* Elewacja chwytaka = `q3 + q4 − q2` (w płaszczyźnie ramienia).
* Limity przegubów [°]: J1 ±145, J2 0..170, J3 0..180, J4 −80..90, J5 ±90, J6 ±130.

### Co ramię realnie osiąga (zmierzone IK i niezależnie siatką po przegubach)

| Orientacja chwytaka | Zasięg |
|---|---|
| `free` (solver dobiera nadgarstek) | ok. 0,15–0,7 m od osi bazy, z ≈ 0,03–0,5 m |
| `down` (w dół) | tylko nisko: z ≲ 0,14 m, x ≈ 0,1–0,45 m |
| `forward` (poziomo) | tylko wyżej: z ≳ 0,15 m |

To wynika z geometrii (długi chwytak, limit zgięcia nadgarstka ±80°), a nie z błędu. Jeśli cel
jest odrzucany, to zwykle słusznie.

## Instalacja

Środowisko `rebot` ma już `motorbridge`, `numpy`, `mcp<2`, `pytest`. Środowisko
przeciekało pakietami z `~/.local`, więc **zawsze uruchamiaj z `PYTHONNOUSERSITE=1`**:

```bash
conda activate rebot   # or any Python 3.10+ environment with the requirements installed
export PYTHONNOUSERSITE=1
cd rebot_b601   # from the repository root
python -m pytest tests -q          # 29 testów, ~35 s, bez sprzętu
```

## Przed pierwszym uruchomieniem na sprzęcie

1. **Zatrzymaj `motorbridge-gateway`** (zajmuje `can0`) i nie uruchamiaj Motorbridge Studio w trakcie.
2. Ustaw `can0` (osobny terminal, wymaga sudo); po zatrzymaniu gatewaya interfejs wrócił do DOWN:
   ```bash
   sudo modprobe peak_usb
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000 restart-ms 100
   sudo ip link set can0 up
   ```
3. Ramię w pozie domowej (złożone), miejsce wokół wolne, ręka przy zasilaniu.

## Skrypty (CLI)

```bash
python -m rebot_b601 ik 0.30 0.05 0.08 --approach down     # offline, zawsze bezpieczne
python -m rebot_b601 state                                  # odczyt (silniki nie są włączane)
python -m rebot_b601 xyz 0.30 0.05 0.08 --approach down --linear --speed 0.2
python -m rebot_b601 joints 0 20 20 0 0 0 --speed 0.2
python -m rebot_b601 grip 0.5
python -m rebot_b601 home
python -m rebot_b601 repl                                   # sesja interaktywna, ramię zostaje pod napięciem
```

Te same komendy mają wrappery w `scripts/` (`move_xyz.py`, `ik_offline.py`, `read_state.py`, `home.py`, `repl.py`).
Każda komenda ruchu ma `--simulate` (fałszywe ramię, bez CAN) i pyta o potwierdzenie (`--yes` je pomija).
Domyślnie po ruchu `xyz`/`joints` stoją 3 s w celu, potem wracają do domu i wyłączają moment
(`--after leave` zostawia silniki włączone, ale wtedy nikt ich już nie steruje, a zachowanie
firmware RS bez ramek CAN nie jest zweryfikowane). Do serii ruchów używaj `repl`.
`Ctrl+C` w trakcie ruchu zatrzymuje go i trzyma pozycję.

Polecenia `repl`: `xyz X Y Z [down|forward|up|free] [linear]`, `rel DX DY DZ`, `plan X Y Z [approach]`,
`joints a1..a6`, `home`, `grip 0..1`, `state`, `stop`, `off` (awaryjne wyłączenie momentu: ramię opadnie), `quit`.

## Serwer MCP (dla agenta)

Rejestracja w Claude Code (najpierw z symulatorem, `REBOT_DRY_RUN=1`):

```bash
claude mcp add rebot-b601 \
  -e PYTHONNOUSERSITE=1 -e PYTHONPATH=<absolute path to this folder> -e REBOT_DRY_RUN=1 \
  -- <path to the python of your env> -m rebot_b601.mcp_server
```

Gdy chcesz sterować prawdziwym ramieniem: usuń `-e REBOT_DRY_RUN=1` (albo agent woła `arm_connect` z `simulate=false`).

| Narzędzie | Działanie |
|---|---|
| `arm_info` | fakty o ramieniu, układ współrzędnych, limity, zasięg (bez ruchu) |
| `arm_connect(simulate, enable_torque)` | połączenie; włącza silniki i trzyma obecną pozę |
| `arm_status` | kąty przegubów, xyz TCP, chwytak, temperatura, ewentualny błąd |
| `arm_plan_xyz(x,y,z,approach,linear)` | sprawdza osiągalność **bez ruchu** |
| `arm_move_to_xyz(x,y,z,approach,linear,speed_scale)` | jedź do punktu |
| `arm_move_relative(dx,dy,dz,…)` | przesunięcie względne (domyślnie po prostej) |
| `arm_move_joints(joints_deg,speed_scale)` | kąty 6 przegubów |
| `arm_home` | poza zerowa |
| `arm_gripper(opening)` | 0 = zamknięty, 1 = otwarty |
| `arm_stop` | zatrzymaj i trzymaj pozycję |
| `arm_emergency_disable` | **tylko awaria**: wyłącza moment, ramię opada |
| `arm_disconnect(go_home)` | wróć do domu, wyłącz moment, zwolnij CAN |

Odmowy zwracane są jako `{"ok": false, "error": "..."}` z czytelnym powodem. Ruchy wykonują się w wątku,
więc `arm_stop` działa w trakcie trwającego ruchu. Gdy serwer się kończy, ramię wraca do domu i wyłącza moment
(`REBOT_ON_EXIT=home|off|leave`).

## Wirtualny klon (podgląd 3D w przeglądarce)

`rebot_b601/viewer.py` serwuje na `http://127.0.0.1:8765/` widok 3D ramienia, który na żywo pokazuje stan `Arm`
(symulowany albo zmierzony z prawdziwego ramienia). Widok tylko odczytuje stan, niczego nie steruje.

```bash
python -m rebot_b601 fetch-assets      # opcjonalnie: ponowne pobranie siatek CAD i three.js (są już w repo, rebot_b601/viewer_assets/, ok. 36 MB)
```

* **Z siatkami CAD:** prawdziwy model (WebGL / three.js), kolory jak na ramieniu, osie TCP na końcówce chwytaka,
  fioletowy znacznik celu, ślad TCP. Siatki pobierane są ze Studio (nie są częścią repo, katalog jest w `.gitignore`;
  to cudze CAD Seeed, nie redystrybuuj). Bez siatek strona wraca do prostego szkieletu kinematycznego.
* **Serwer MCP:** w trybie `REBOT_DRY_RUN=1` podgląd startuje sam (port z `REBOT_VIEWER_PORT`, domyślnie 8765;
  na prawdziwym ramieniu jest domyślnie wyłączony, włącz `REBOT_VIEWER_PORT=8765`). Adres zwraca `arm_info`.
  Na prawdziwym ramieniu to cyfrowy bliźniak zasilany zmierzonymi kątami.
* **CLI:** `python -m rebot_b601 repl --simulate --viewer` (lub `xyz ... --viewer 8765`).
* Sterowanie widokiem: przeciąganie (orbita), prawy przycisk (przesuwanie), kółko (zoom), dwuklik (reset).

## Zabezpieczenia

* Limity przegubów: cel poza limitem jest **odrzucany**, nie przycinany po cichu.
* Prędkość: profil min-jerk; `speed_scale` domyślnie 0,3, twardy limit serwera 0,6 (`REBOT_MAX_SPEED`); szczyt na skali 1 to 30–90°/s zależnie od przegubu.
* Skrzynka robocza TCP (±0,6 m, z 0,03–0,7 m) i prosty strażnik stołu/bazy sprawdzany wzdłuż całej ścieżki (`REBOT_Z_MIN`). **Brak unikania kolizji z przedmiotami.**
* Błąd śledzenia: przegub > 12° od zadanej pozycji przez 0,4 s (blokada, kolizja) przerywa ruch, trzyma pozę i blokuje dalsze ruchy do `clear_fault()` (moment zostaje włączony, trzyma bieżącą pozę) albo ponownego połączenia. Komunikat podaje kąt zadany i zmierzony.
* Temperatura MOSFET: 125 °C przerywa ruch, 135 °C wyłącza moment. Utrata sprzężenia zwrotnego > 0,3 s także przerywa.
* Przy włączaniu: pozycja odniesienia silnika jest ustawiana na **bieżącą** pozę *przed* włączeniem (bez skoku), a odczyt poza zakresem o >15° blokuje włączenie (brak kalibracji zera).
* Chwytak: sterowanie momentem jak w follower Seeed (limit 3 Nm przy ruchu, 1 Nm przy trzymaniu). Mapowanie `opening` → kąt silnika (`REBOT_GRIPPER_OPEN_DEG`, domyślnie 240°) jest przybliżone: sprawdź własny chwytak.

## Zmienne środowiskowe

`REBOT_CAN_CHANNEL` (can0), `REBOT_SEND` (`pos_vel`: `send_pos_vel` z motorbridge dla każdego silnika w każdym takcie, jak wcześniej; domyślnie: tylko zmienione pozycje zadane, zob. `HardwareBackend`), `REBOT_DRY_RUN`, `REBOT_DEFAULT_SPEED`, `REBOT_MAX_SPEED`, `REBOT_Z_MIN`,
`REBOT_MOTOR_VLIM`, `REBOT_TRACKING_ERR_DEG`, `REBOT_GRIPPER_OPEN_DEG`, `REBOT_GRIPPER_TORQUE`,
`REBOT_DISABLE_GRIPPER`, `REBOT_ON_EXIT`. Pozostałe stałe: `rebot_b601/config.py`.

## Znane ograniczenia

* Niezweryfikowane na sprzęcie (patrz wyżej): zwłaszcza sekwencja włączania (`0x7016` = pozycja odniesienia) i zachowanie silników, gdy proces przestaje wysyłać ramki.
* Strażnik stołu zakłada, że płyta bazy leży na stole (z = 0) i nie zna przedmiotów na nim.
* Ruch po prostej (`linear`) przerywa się na osobliwościach lub przy dużym skoku przegubów; wtedy użyj ścieżki w przestrzeni przegubów.
* Brak samokolizji ramienia z samym sobą poza prostą kontrolą punktów.
