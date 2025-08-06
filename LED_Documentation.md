# Guía de Control de LEDs para Robots NAO y Pepper

Este documento explica cómo controlar los LEDs de los robots NAO y Pepper usando el tópico de ROS2 proporcionado por el nodo `naoqi_miscellaneous_node`.

## Cómo Usar

Para controlar los LEDs, debes publicar un mensaje en el tópico `~/set_leds` (relativo al namespace del nodo, por ejemplo, `/naoqi_miscellaneous_node/set_leds`).

- **Tópico:** `~/set_leds`
- **Tipo de Mensaje:** `naoqi_utilities_msgs/msg/LedParameters`

### Estructura del Mensaje

```plaintext
string name         # Nombre del LED o grupo de LEDs.
uint8 red           # Valor del canal rojo (0-255).
uint8 green         # Valor del canal verde (0-255).
uint8 blue          # Valor del canal azul (0-255).
float32 duration    # Duración de la transición en segundos.
```

### Ejemplo de Comando

Para encender los LEDs de la cara del robot en color azul durante 1.5 segundos:

```bash
ros2 topic pub /naoqi_miscellaneous_node/set_leds naoqi_utilities_msgs/msg/LedParameters "{name: 'FaceLeds', red: 0, green: 0, blue: 255, duration: 1.5}"
```

---

## Grupos de LEDs Relevantes

A continuación se listan los nombres de grupo (`name`) más comunes y útiles para cada robot.

### 🤖 NAO

| Nombre del Grupo (`name`) | Descripción                               |
| ------------------------- | ----------------------------------------- |
| `FaceLeds`                | Ambos ojos.                               |
| `LeftFaceLeds`            | Ojo izquierdo.                            |
| `RightFaceLeds`           | Ojo derecho.                              |
| `ChestLeds`               | El LED del botón del pecho.               |
| `BrainLeds`               | Todos los LEDs en la parte superior de la cabeza. |
| `EarLeds`                 | Ambos oídos.                              |
| `LeftEarLeds`             | Oído izquierdo.                           |
| `RightEarLeds`            | Oído derecho.                             |
| `FeetLeds`                | Los LEDs en ambos pies.                   |
| `AllLeds`                 | Absolutamente todos los LEDs del robot.   |

### 🤖 Pepper

| Nombre del Grupo (`name`) | Descripción                               |
| ------------------------- | ----------------------------------------- |
| `EyeLeds`                 | Ambos ojos.                               |
| `LeftEyeLeds`             | Ojo izquierdo.                            |
| `RightEyeLeds`            | Ojo derecho.                              |
| `ShoulderLeds`            | Los LEDs de ambos hombros.                |
| `LeftShoulderLeds`        | Hombro izquierdo.                         |
| `RightShoulderLeds`       | Hombro derecho.                           |
| `EarLeds`                 | Los LEDs de los oídos.                    |
| `AllLeds`                 | Todos los LEDs del robot.                 |

---

### Notas Adicionales

- **Transiciones:** El campo `duration` permite crear efectos de encendido y apagado suaves (fade). Un valor de `0.0` cambia el color instantáneamente.
- **Colores:** Para apagar un LED, simplemente envía un mensaje con los valores de `red`, `green` y `blue` en `0`.
- **Nombres Específicos:** Además de los grupos, puedes usar nombres de LEDs individuales (ej. `RightFaceLed1`), pero los grupos son generalmente más útiles para efectos visuales.