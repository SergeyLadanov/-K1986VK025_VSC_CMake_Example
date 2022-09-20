cmake_minimum_required(VERSION 3.21) # Проверка версии CMake.
									# Если версия установленой программы
									# старее указаной, произайдёт аварийный выход.

project(Submodule)				# Название проекта

set(SOURCE_LIB 
    Src/test.cpp
)		# Установка переменной со списком исходников

include_directories(
    Inc
)

add_library(Submodule STATIC ${SOURCE_LIB})# Создание статической библиотеки