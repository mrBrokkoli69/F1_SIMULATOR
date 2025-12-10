# Компилятор и флаги
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
NCURSES_FLAGS = -lncurses -ltinfo
MATH_FLAGS = -lm

# Целевые файлы
TARGET = monza_simulator
OBJS = curve.o F1_Physics.o monza.o

# Правило по умолчанию
all: $(TARGET)

# Основная цель
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(NCURSES_FLAGS) $(MATH_FLAGS)

# Компиляция отдельных модулей
curve.o: curve.cpp curve.h
	$(CXX) $(CXXFLAGS) -c curve.cpp -o curve.o

F1_Physics.o: F1_Physics.cpp F1_Physics.h
	$(CXX) $(CXXFLAGS) -c F1_Physics.cpp -o F1_Physics.o

monza.o: monza.cpp curve.h F1_Physics.h
	$(CXX) $(CXXFLAGS) -c monza.cpp -o monza.o

# Очистка
clean:
	rm -f $(OBJS) $(TARGET)

# Перекомпиляция
rebuild: clean all




# Запуск симулятора (после компиляции)
run: $(TARGET)
	./$(TARGET)

# Отладочная сборка
debug: CXXFLAGS += -g -DDEBUG
debug: rebuild

# Установка зависимостей (для Ubuntu/Debian)
install-deps:
	sudo apt-get update
	sudo apt-get install libncurses5-dev libncursesw5-dev

# Альтернативная установка зависимостей (для Arch Linux)
install-deps-arch:
	sudo pacman -S ncurses

# Помощь
help:
	@echo "Доступные цели:"
	@echo "  all       - компиляция проекта (по умолчанию)"
	@echo "  clean     - удаление скомпилированных файлов"
	@echo "  rebuild   - полная перекомпиляция"
	@echo "  run       - запуск симулятора"
	@echo "  debug     - отладочная сборка"
	@echo "  install-deps - установка зависимостей (Ubuntu/Debian)"
	@echo "  install-deps-arch - установка зависимостей (Arch Linux)"
	@echo "  help      - это сообщение"

.PHONY: all clean rebuild run debug install-deps install-deps-arch help
