# behaviorfleets

- Hacer Any
- Se publica el árbol en un topic /mission_command
- Todos los nodos remotos suscritos reciben la misión (con su tipo para decidir si la pueden ejecutar)
- Si el robot puede ejecutar la misión publica su identificador en /mission_answ
- El nodo origen se queda con el identificador del primero que responde y le envía el árbol por /mission_command/<id>
- El nodo remoto, al recibir el árbol, ejecuta el comportamiento
- Los nodos que no han recibido el árbol (no se ejecuta su callback) siguen atentos a nuevas misiones que puedan llegar