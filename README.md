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
* **Marker**. *Marker topic:* marcas_sonares
* **Map**.  *Topic:* occupancy_marker
* **Map**.  *Topic:* occupancy_marker_marcas

En el **Grid** en rviz, modificar los siguientes parámetros para que las celdas
coincidan con el código:
* **Reference Frame**. odom
* **Plane Cell Count**. Incrementar al número máximo de celdas horizontales o
                        verticales. Ej: 31.
* **Cell Size**. Al tamaño de la resolución en el código.  Ej: 0.3.
* **Offset**. La mitad de la resolución tanto en x. Ej: 0.15.
