# Extensión Inteligente Hydrate

Este es el firmware usado por la extensión inteligente Hydrate, que es capaz de detectar y medir el consumo de líquidos desde una botella conectada. 

Para informar sobre un error, sugerir una características o hacer una pregunta, es posible usar la sección de [issues](https://github.com/Frozen-Burrito/hydrate-extension-inteligente/issues/new/choose). 

## Características Principales

La extensión inteligente Hydrate está orientada a cualquier persona que desee conocer mejor su consumo de agua a lo largo del día o que requiera apoyo para formar un hábito saludable de consumo de agua.

La extensión logra esto a través de sus características principales, que incluyen:

* Inferencia de hidratación, basada en el movimiento del dispositivo y el peso de la botella sobre él.
* Compatibilidad con casi cualquier botella de agua reutilizable.
* Integración inalámbrica con el dispositivo móvil, usando la app Hydrate.
* Optimización de uso de energía, para soportar períodos de uso extendidos.


## Estructura del Repositorio

El proyecto **hydrate-extension-inteligente** está desarrollado usando el lenguaje C.  Contiene dos directorios principales: [main](main) y [components](components). Todo el firmware se encuentra en estos dos directorios, en varios archivos `.c` y `.h`.

Los proyectos basados en ESP-IDF son compilados usando CMake. La configuración de compilación del proyecto se encuentra en los múltiples archivos `CMakeLists.txt`,
que proporcionan el conjunto de directivas e instrucciones para describir los archivos fuente y los targets
(ejecutable, librería, o ambos). 

Abajo se muestra una explicación breve de los archivos encontrados en el directorio del proyecto.

```
├── CMakeLists.txt
├── .gitignore
├── main
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   └── main.c
├── components
│   ├── storage
|   |   ├── include
|   |   └── storage.c
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.

## Recursos de Apoyo
