# TEMPOMAT - Sterownik Silnika DC na STM32

## O Projekcie

Projekt realizuje układ tempomatu (stabilizacji prędkości obrotowej) dla silnika prądu stałego (DC) z wykorzystaniem mikrokontrolera **STM32L4A6ZG**. System oparty jest na pętli sprzężenia zwrotnego z regulatorem **PID**, który dopasowuje napięcie zasilania silnika (PWM) w celu utrzymania zadanej prędkości, niezależnie od obciążenia.

## Funkcjonalności

*   **Precyzyjny pomiar prędkości**: Wykorzystanie enkodera inkrementalnego oraz sprzętowego licznika Timer 2 do zliczania impulsów.
*   **Regulator PID**: Zaimplementowany cyfrowy algorytm PID z ochroną przed nasyceniem integratora (anti-windup) i filtracją dolnoprzepustową pomiarów.
*   **Zdalne Sterowanie (YK04)**: Obsługa 4-kanałowego pilota radiowego do szybkiej zmiany nastaw.
*   **Interfejs UART**:
    *   Wysyłanie telemetrii na żywo (prędkość, uchyb, wysterowanie).
    *   Możliwość ręcznego wpisania zadanej prędkości z terminala.
*   **Wyświetlacz LCD**: Prezentacja aktualnej prędkości i uchybu regulacji w czasie rzeczywistym.

## Schemat Połączeń (Hardware)

### 1. Elementy Wykonawcze (Silnik)
*   **Mostek H (Sterownik silnika)**:
    *   **PWM (Prędkość)**: `TIM5_CH2` (Sygnał sterujący napięciem).
    *   **Kierunek (IN1)**: `PF8`.
    *   **Kierunek (IN2)**: `PF9`.

### 2. Pomiary (Enkoder)
*   **Enkoder Inkrementalny**:
    *   Podłączony do wejść Timera 2 (`TIM2`) w trybie *Encoder Mode*.

### 3. Sterowanie Radiowe (Odbiornik YK04)
Moduł podłączony do portu **GPIOD**:
*   **Przycisk A** (`PD0`): Zwiększ prędkość o **+10 rad/s**.
*   **Przycisk B** (`PD1`): Zmniejsz prędkość o **-10 rad/s**.
*   **Przycisk C** (`PD2`): **STOP** (Reset prędkości i błędu całkującego).
*   **Przycisk D** (`PD3`): Ustaw prędkość preset **100 rad/s**.

### 4. Interfejs Użytkownika
*   **Wyświetlacz LCD (2x16)**: Magistrala I2C1.
*   **UART (USB)**: LPUART1 (przez ST-Link) - prędkość **115200 bps**.

## Parametry Techniczne

| Parametr | Wartość |
| :--- | :--- |
| **Mikrokontroler** | STM32L4A6ZG |
| **Napięcie zasilania silnika** | 12 V |
| **Częstotliwość próbkowania** | 50 Hz (20 ms) |
| **Rozdzielczość enkodera** | 48 impulsów/obrót |
| **Maks. wypełnienie PWM** | 2399 (ARR) |

## Konfiguracja Regulatora PID
Aktualne nastawy regulatora dobrane eksperymentalnie:
*   **Kp (Człon proporcjonalny):** `1.0`
*   **Ki (Człon całkujący):** `0.5`
*   **Kd (Człon różniczkujący):** `0.01`


## Instrukcja Uruchomienia

1.  **Połączenie**: Podłącz płytkę Nucleo do komputera przez USB. Upewnij się, że zasilanie silnika (12V) jest włączone.
2.  **Kompilacja**: Otwórz projekt w STM32CubeIDE i wgraj oprogramowanie.
3.  **Start**:
    *   Po uruchomieniu na LCD pojawi się napis "Start".
    *   Domyślna prędkość początkowa to **5.0 rad/s**.
4.  **Kontrola**:
    *   Użyj pilota YK04 do zmiany prędkości.
    *   Lub otwórz terminal szeregowy (np. PuTTY, RealTerm) na porcie COM ST-Linka (Baud: 115200) aby podglądać wykresy lub wpisać nową prędkość ręcznie (np. `50.5` + Enter).

## Struktura Katalogów
*   `Core/Src/main.c`: Główna pętla sterowania, obsługa przerwań Timera i logika YK04.
*   `scripts/matlab/`: Skrypty pomocnicze do wyznaczania nastaw PID metodą LQR.
*   `Core/Inc/main.h`: Definicje pinów i stałych.'
*   `scripts/python/`: Skrypty do komunikacji z mikrokontrolerem i wizualizacji danych telemetrycznych.
