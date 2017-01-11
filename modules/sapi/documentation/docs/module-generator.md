# Generador de módulos de la biblioteca

*Para desarrolladores.*

Este generador de módulos tiene que soportar:

- Definir tipos de datos.
    - Tipos simples.
    - Tipos estructurados.
    - Documentación que indica el propósito del tipo de datos.
- Definir atributos (o propiedades) que tendrán cierto tipos de datos. Todos los atributos van a ser privados, accesibles mediante métodos getters y setters.
    - Documentación que indica el propósito del atributo.
- Definir métodos, con ciertos parámetros que tendrán ciertos tipos de datos y un cuerpo de método escrito en lenguaje C. Opcionalmente un métododo tendá además modificadores. Existirán 2 tipos de métodos:
    - Métodos privados. Accesibles sólo por el módulo.
    - Métodos públicos. Accesibles por el módulo y otros módulos.
    - Flag que
    - Documentación que indica el propósito del método.
- Definir un módulo que contendrá: sus tipos de datos, dependencias de tipos externos, dependencias de otros módulos, propiedades y métodos.
    - Documentación que indica el propósito del módulo.
    - Documentación que indica las restricciones de uso según board (tiene un conjunto de boards).
- Generar archivos en lenguaje C (.c y .h), con documentación en formato doxygen.
    - modueleName.h, con headers y declaraciones públicas, independiente del hardware.
    - modueleName.c, dependiente del hardware. Opcionalmente se podrá dividir en dos archivos, uno dependiente del hardware y uno independiente.
- Generar automáticamente getters y setters de atributos.
- Documentación es un objeto que tiene un Dicccionario con asocieciones lenguaje->texto.

