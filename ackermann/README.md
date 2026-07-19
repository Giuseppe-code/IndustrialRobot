# Ackermann Shuttle 3D

Visualizzatore Godot per il controller Python in `../controller/`.

## Funzioni integrate

- Posa planare del cart: `X`, `Y`, `Theta`.
- Peso dinamico del carico: in zona `A` vengono caricati i pacchi, in zona
  `B` vengono scaricati.
- Zona di confidenza circolare configurabile nel notebook tramite
  `zone_radius`.

## Topic DDS

Godot pubblica verso Python:

| Topic | Significato |
| --- | --- |
| `tick` | Passo di sincronizzazione |

Python pubblica verso Godot:

| Topic | Significato |
| --- | --- |
| `X`, `Y` | Posizione planare |
| `Theta` | Orientamento sul piano |
| `PayloadMass` | Massa dei pacchi a bordo |
| `MissionState` | Codice dello stato missione corrente |
| `CargoColorCode` | Codice colore del pacco selezionato |
| `UnloadZoneCode` | Codice della zona di scarico selezionata |

## Configurare A, B e carico

Nel notebook `../controller/test_ackermann_trajectory.ipynb`:

```python
point_a = (0.0, 0.0)
point_b = (5.0, 2.0)
zone_radius = 0.25
packages_to_load = [5.0, 5.0, 5.0]
```

Il cart parte con massa base `10 kg`. Entrando nel raggio della zona `A`
carica `15 kg`, quindi viaggia con massa totale `25 kg`; entrando nel raggio
di `B` scarica e ritorna a `10 kg`.

## Esecuzione

Dalla radice di `RoboticSystemShuttle`:

```bash
python3 -m pip install -r requirements.txt
```

Avviare il progetto `ackermann/` in Godot, poi eseguire il notebook
`controller/test_ackermann_trajectory.ipynb`.
