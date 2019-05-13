# campos_potenciales
Implementación en RVIZ con Kobuki de campos potenciales.

Requiere kobuki_softnode y rviz.

Para ejecutar:

```
roslaunch kobuki_softnode full.launch
rosrun rviz rviz
rosrun campos_potenciales basic_fields
roslaunch kobuki_keyop keyop.launch
```

En rviz agregar los elementos siguientes:

* **RobotModel**.
* **Marker**.  *Marker topic:* visualization_marker
* **Marker**. *Marker topic:* marca_meta
* **Map**.  *Topic:* occupancy_marker
* **Map**.  *Topic:* occupancy_marker_marcas
