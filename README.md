El siguiente procedimiento muestra un diseño detallado del controlador por retroalimentación de estados para mejorar la eficiencia y respuesta dinámica de un sistema PV conectado a la red. Este diseño está relacionado con el artículo que se encuentra en la liga [https://itchihuahua.mx/congreso_electro/](https://itchihuahua.mx/congreso_electro/), y se muestra en el archivo `Dis_Cont_Conv_Boost_V1`.

<img src="https://github.com/user-attachments/assets/80e6a31c-7255-4845-9797-8025dfb569dd" alt="Diagrama del controlador" width="600"/>


Para ejecutar los modelos de simulación primero ejecutar el programa "M_ELECTRO2025_V1" en Matlab y seguido ejecutar el modelo de Simulink "S_ELECTRO2025_V1"

<img src="https://github.com/user-attachments/assets/ce7bf9b0-f72e-4707-a8a0-5804eca8acb2" alt="Imagen 1" width="600"/>


<img src="https://github.com/user-attachments/assets/ed17a044-5a49-4bac-a78b-121df6ca4d88" alt="Imagen 2" width="600"/>

S_ELECTRO_2025_MTET_NoInt_EnvDatos_V1 se programa en la tarjeta F28069M y S_GRAF_ELECTRO_2025_MTET_NoInt_EnvDatos_V1 se ejecuta en Matlab/Simulink para verificar la funcionalidad del código (controlador) mediante la graficacion de la corriente del arreglo PV (ipv), la corriente en el punto MPP (Imppt) y el voltaje del bus de dc (vdc). 

<img src="https://github.com/user-attachments/assets/daac86c4-5871-4736-b520-d30a148a83f1" alt="Imagen 3" width="600"/>

<img src="https://github.com/user-attachments/assets/f6ded404-48f7-472c-a90a-52e55cd55756" alt="Imagen 4" width="600"/>

S_ELECTRO_2025_MTET_NoInt_V1 es el código (controlador) que se baja en la tarjeta F28069M para la medición de tiempo de ejecución. Este código es el que finalmente se deja en la tarjeta con fines de control.

<img src="https://github.com/user-attachments/assets/4c0b8683-e914-4b7e-847a-4cf7f838d875" alt="Imagen 5" width="600"/>



