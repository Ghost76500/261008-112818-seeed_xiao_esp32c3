#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================
//     正常振动频率: 80 ~ 180 Hz；FFT仍观察到300 Hz用于超频判断
//
// Current UART output:
//     VIB=1,FREQ=235.31Hz,STARTSTOP=0.000Hz,RATIO=69.19（RATIO是FFT谱峰相对背景的强度，>10数据可信度高）
//
// Hardware:
//     ESP32-C3
//     MPU6050
// ============================================================

#define SDA_PIN                 9
#define SCL_PIN                 8

#define MPU_ADDR                0x68

#define SERIAL_BAUD             115200


// LED pins
#define GREEN_LED_PIN           0
#define YELLOW_LED_PIN          1


// ============================================================
// Sampling configuration
// ============================================================

#define SAMPLE_COUNT            1024

//采样频率
#define SAMPLE_PERIOD_US        1000

//正常检测范围
#define FREQ_MIN                80.0f
#define FREQ_MAX                180.0f

//FFT仍观察到300Hz，用于判断是否超过180Hz
#define SEARCH_FREQ_MAX         300.0f


// ============================================================
// Short-time envelope
//
// 32 samples @ 1000 Hz = 32 ms
// ============================================================

#define ENV_BLOCK_SIZE          32

#define ENV_BLOCK_COUNT         \
    (SAMPLE_COUNT / ENV_BLOCK_SIZE)


// ============================================================
// Vibration detection thresholds
//
// Based on current measurements:
//
// Quiet:
//     envelope approximately 50~100
//
// Strong vibration:
//     envelope can reach approximately 270~330
//
// Hysteresis:
//
//     OFF threshold = 120
//     ON  threshold = 180
// ============================================================

#define VIB_ON_THRESHOLD        180.0f
#define VIB_OFF_THRESHOLD       120.0f


// ============================================================
// Spectrum validity
//
// Quiet Peak Ratio:
//     usually about 2~4
//
// Real vibration:
//     can be much greater than 5
// ============================================================

#define MIN_PEAK_RATIO          5.0f

//启停范围
#define STARTSTOP_MAX_HZ        3.0f
#define STARTSTOP_TIMEOUT_MS    3000


// ============================================================
// MPU6050 registers
// ============================================================
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C

#define REG_ACCEL_XOUT_H        0x3B

#define REG_PWR_MGMT_1          0x6B
#define REG_WHO_AM_I            0x75

// ============================================================
// Sample buffers
// ============================================================
int16_t axBuffer[SAMPLE_COUNT];
int16_t ayBuffer[SAMPLE_COUNT];
int16_t azBuffer[SAMPLE_COUNT];

// ============================================================
// FFT buffers
// ============================================================
float fftReal[SAMPLE_COUNT];
float fftImag[SAMPLE_COUNT];

// ============================================================
// FFT result
// ============================================================
struct FFTResult
{
    char axis;

    float rms;

    int peakBin;

    float peakFrequency;

    float peakMagnitude;

    float backgroundMagnitude;

    float peakRatio;
};

// ============================================================
// System states
// ============================================================
bool vibrationState = false;
// Current detected carrier frequency
float mainFrequency = 0.0f;
// Current start/stop frequency
float startStopFrequency = 0.0f;
// Rising-edge timing
uint32_t lastRisingEdgeMs = 0;
uint32_t previousRisingEdgeMs = 0;

// ============================================================
// MPU6050 register write
// ============================================================

bool writeRegister(
    uint8_t reg,
    uint8_t value
)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

// ============================================================
// MPU6050 register read
// ============================================================

bool readRegister(
    uint8_t reg,
    uint8_t &value
)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);


    if (
        Wire.endTransmission(false)
        != 0
    )
    {
        return false;
    }


    if (
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)1,
            (uint8_t)true
        )
        != 1
    )
    {
        return false;
    }


    value = Wire.read();

    return true;
}

// ============================================================
// Read acceleration X/Y/Z
// ============================================================
bool readAcceleration(
    int16_t &ax,
    int16_t &ay,
    int16_t &az
)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(REG_ACCEL_XOUT_H);


    if (
        Wire.endTransmission(false)
        != 0
    )
    {
        return false;
    }


    uint8_t count =
        Wire.requestFrom(
            (uint8_t)MPU_ADDR,
            (uint8_t)6,
            (uint8_t)true
        );


    if (count != 6)
    {
        return false;
    }


    ax =
        (int16_t)(
            (Wire.read() << 8)
            |
            Wire.read()
        );


    ay =
        (int16_t)(
            (Wire.read() << 8)
            |
            Wire.read()
        );


    az =
        (int16_t)(
            (Wire.read() << 8)
            |
            Wire.read()
        );


    return true;
}

// ============================================================
// MPU6050 initialization
// ============================================================

bool initMPU6050()
{
    uint8_t who = 0;


    // --------------------------------------------------------
    // Check MPU6050
    // --------------------------------------------------------

    if (
        !readRegister(
            REG_WHO_AM_I,
            who
        )
    )
    {
        return false;
    }


    if (who != 0x68)
    {
        return false;
    }

    // --------------------------------------------------------
    // Wake MPU6050
    // CLKSEL = X-axis gyro PLL
    // --------------------------------------------------------

    if (
        !writeRegister(
            REG_PWR_MGMT_1,
            0x01
        )
    )
    {
        return false;
    }


    delay(100);


    // --------------------------------------------------------
    // CONFIG = 0
    //
    // Wide accelerometer bandwidth.
    // Current test vibration is around 235 Hz,
    // so we keep wide bandwidth.
    // --------------------------------------------------------

    if (
        !writeRegister(
            REG_CONFIG,
            0x00
        )
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Accelerometer ±2g
    //
    // Sensitivity:
    // 16384 LSB/g
    // --------------------------------------------------------
    if (
        !writeRegister(
            REG_ACCEL_CONFIG,
            0x00
        )
    )
    {
        return false;
    }

    // --------------------------------------------------------
    // Gyroscope ±250 dps
    //
    // Gyroscope is currently not used in FFT.
    // --------------------------------------------------------

    if (
        !writeRegister(
            REG_GYRO_CONFIG,
            0x00
        )
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Sample divider
    // --------------------------------------------------------

    if (
        !writeRegister(
            REG_SMPLRT_DIV,
            0x00
        )
    )
    {
        return false;
    }


    delay(100);

    return true;
}


// ============================================================
// Capture 1024 acceleration samples
// ============================================================
float captureSamples(
    int &i2cErrors,
    uint32_t &firstSampleUs,
    uint32_t &lastSampleUs
)
{
    i2cErrors = 0;

    firstSampleUs = 0;
    lastSampleUs = 0;


    uint32_t nextSampleTime =
        micros();


    for (
        int i = 0;
        i < SAMPLE_COUNT;
        i++
    )
    {
        // ----------------------------------------------------
        // Precise 1 ms sampling interval
        // ----------------------------------------------------

        while (
            (int32_t)(
                micros()
                -
                nextSampleTime
            ) < 0
        )
        {
            yield();
        }


        uint32_t sampleTime =
            micros();


        if (i == 0)
        {
            firstSampleUs =
                sampleTime;
        }


        if (
            i ==
            SAMPLE_COUNT - 1
        )
        {
            lastSampleUs =
                sampleTime;
        }


        // ----------------------------------------------------
        // MPU6050 read
        // ----------------------------------------------------

        if (
            !readAcceleration(
                axBuffer[i],
                ayBuffer[i],
                azBuffer[i]
            )
        )
        {
            i2cErrors++;


            // If one read fails,
            // reuse previous sample instead of inserting zero.

            if (i > 0)
            {
                axBuffer[i] =
                    axBuffer[i - 1];

                ayBuffer[i] =
                    ayBuffer[i - 1];

                azBuffer[i] =
                    azBuffer[i - 1];
            }
            else
            {
                axBuffer[i] = 0;
                ayBuffer[i] = 0;
                azBuffer[i] = 0;
            }
        }


        nextSampleTime +=
            SAMPLE_PERIOD_US;
    }


    // --------------------------------------------------------
    // N samples contain N-1 time intervals
    // --------------------------------------------------------

    uint32_t elapsed =
        lastSampleUs
        -
        firstSampleUs;


    if (elapsed == 0)
    {
        return 0.0f;
    }


    return
        (
            (SAMPLE_COUNT - 1)
            *
            1000000.0f
        )
        /
        elapsed;
}


// ============================================================
// Calculate AC RMS
// ============================================================

float calculateRMS(
    int16_t *buffer,
    float &mean
)
{
    int64_t sum = 0;


    for (
        int i = 0;
        i < SAMPLE_COUNT;
        i++
    )
    {
        sum += buffer[i];
    }


    mean =
        (float)sum
        /
        SAMPLE_COUNT;


    double sumSquare = 0.0;


    for (
        int i = 0;
        i < SAMPLE_COUNT;
        i++
    )
    {
        float value =
            buffer[i]
            -
            mean;


        sumSquare +=
            (double)value
            *
            (double)value;
    }


    return sqrt(
        sumSquare
        /
        SAMPLE_COUNT
    );
}


// ============================================================
// Radix-2 FFT
// ============================================================

void performFFT(
    float *real,
    float *imag,
    int n
)
{
    int j = 0;


    // --------------------------------------------------------
    // Bit reversal
    // --------------------------------------------------------

    for (
        int i = 1;
        i < n;
        i++
    )
    {
        int bit =
            n >> 1;


        while (j & bit)
        {
            j ^= bit;

            bit >>= 1;
        }


        j ^= bit;


        if (i < j)
        {
            float temp;


            temp = real[i];

            real[i] =
                real[j];

            real[j] =
                temp;


            temp = imag[i];

            imag[i] =
                imag[j];

            imag[j] =
                temp;
        }
    }


    // --------------------------------------------------------
    // FFT stages
    // --------------------------------------------------------

    for (
        int len = 2;
        len <= n;
        len <<= 1
    )
    {
        float angle =
            -2.0f
            *
            PI
            /
            len;


        float wLenReal =
            cosf(angle);


        float wLenImag =
            sinf(angle);


        for (
            int i = 0;
            i < n;
            i += len
        )
        {
            float wReal =
                1.0f;


            float wImag =
                0.0f;


            for (
                int k = 0;
                k < len / 2;
                k++
            )
            {
                int evenIndex =
                    i + k;


                int oddIndex =
                    i
                    +
                    k
                    +
                    len / 2;


                float oddReal =
                    real[oddIndex]
                    *
                    wReal
                    -
                    imag[oddIndex]
                    *
                    wImag;


                float oddImag =
                    real[oddIndex]
                    *
                    wImag
                    +
                    imag[oddIndex]
                    *
                    wReal;


                float evenReal =
                    real[evenIndex];


                float evenImag =
                    imag[evenIndex];


                real[evenIndex] =
                    evenReal
                    +
                    oddReal;


                imag[evenIndex] =
                    evenImag
                    +
                    oddImag;


                real[oddIndex] =
                    evenReal
                    -
                    oddReal;


                imag[oddIndex] =
                    evenImag
                    -
                    oddImag;


                float newWReal =
                    wReal
                    *
                    wLenReal
                    -
                    wImag
                    *
                    wLenImag;


                float newWImag =
                    wReal
                    *
                    wLenImag
                    +
                    wImag
                    *
                    wLenReal;


                wReal =
                    newWReal;


                wImag =
                    newWImag;
            }
        }
    }
}


// ============================================================
// FFT magnitude
// ============================================================

float getMagnitude(int bin)
{
    return sqrtf(
        fftReal[bin]
        *
        fftReal[bin]
        +
        fftImag[bin]
        *
        fftImag[bin]
    );
}


// ============================================================
// Analyze one acceleration axis
// ============================================================

FFTResult analyzeAxis(
    char axis,
    int16_t *buffer,
    float fs
)
{
    FFTResult result;


    result.axis =
        axis;


    float mean =
        0.0f;


    result.rms =
        calculateRMS(
            buffer,
            mean
        );


    // --------------------------------------------------------
    // Remove DC + Hann window
    // --------------------------------------------------------

    for (
        int i = 0;
        i < SAMPLE_COUNT;
        i++
    )
    {
        float ac =
            buffer[i]
            -
            mean;


        float window =
            0.5f
            *
            (
                1.0f
                -
                cosf(
                    2.0f
                    *
                    PI
                    *
                    i
                    /
                    (SAMPLE_COUNT - 1)
                )
            );


        fftReal[i] =
            ac
            *
            window;


        fftImag[i] =
            0.0f;
    }


    // --------------------------------------------------------
    // FFT
    // --------------------------------------------------------

    performFFT(
        fftReal,
        fftImag,
        SAMPLE_COUNT
    );


    // --------------------------------------------------------
    // Search 80~300 Hz; 180~300 Hz is only used for over-frequency detection
    // --------------------------------------------------------

    int minBin =
        (int)ceilf(
            FREQ_MIN
            *
            SAMPLE_COUNT
            /
            fs
        );


    int maxBin =
        (int)floorf(
            SEARCH_FREQ_MAX
            *
            SAMPLE_COUNT
            /
            fs
        );


    int peakBin =
        minBin;


    float peakMagnitude =
        0.0f;


    // --------------------------------------------------------
    // Find strongest spectral component
    // --------------------------------------------------------

    for (
        int k = minBin;
        k <= maxBin;
        k++
    )
    {
        float magnitude =
            getMagnitude(k);


        if (
            magnitude >
            peakMagnitude
        )
        {
            peakMagnitude =
                magnitude;


            peakBin =
                k;
        }
    }


    result.peakBin =
        peakBin;


    result.peakMagnitude =
        peakMagnitude;


    // --------------------------------------------------------
    // Calculate spectral background
    //
    // Ignore peak +/-3 bins.
    // --------------------------------------------------------

    double backgroundSum =
        0.0;


    int backgroundCount =
        0;


    for (
        int k = minBin;
        k <= maxBin;
        k++
    )
    {
        if (
            k >= peakBin - 3
            &&
            k <= peakBin + 3
        )
        {
            continue;
        }


        backgroundSum +=
            getMagnitude(k);


        backgroundCount++;
    }


    if (
        backgroundCount > 0
    )
    {
        result.backgroundMagnitude =
            backgroundSum
            /
            backgroundCount;
    }
    else
    {
        result.backgroundMagnitude =
            0.0f;
    }


    // --------------------------------------------------------
    // Peak / background
    // --------------------------------------------------------

    if (
        result.backgroundMagnitude
        >
        0.000001f
    )
    {
        result.peakRatio =
            result.peakMagnitude
            /
            result.backgroundMagnitude;
    }
    else
    {
        result.peakRatio =
            0.0f;
    }


    // --------------------------------------------------------
    // Parabolic peak interpolation
    // --------------------------------------------------------

    float delta =
        0.0f;


    if (
        peakBin > minBin
        &&
        peakBin < maxBin
    )
    {
        float left =
            getMagnitude(
                peakBin - 1
            );


        float center =
            getMagnitude(
                peakBin
            );


        float right =
            getMagnitude(
                peakBin + 1
            );


        float denominator =
            left
            -
            2.0f
            *
            center
            +
            right;


        if (
            fabsf(denominator)
            >
            0.000001f
        )
        {
            delta =
                0.5f
                *
                (left - right)
                /
                denominator;
        }


        if (delta > 0.5f)
        {
            delta =
                0.5f;
        }


        if (delta < -0.5f)
        {
            delta =
                -0.5f;
        }
    }


    result.peakFrequency =
        (
            peakBin
            +
            delta
        )
        *
        fs
        /
        SAMPLE_COUNT;


    return result;
}


// ============================================================
// Calculate vibration envelope for one 32 ms block
// ============================================================

float calculateEnvelopeBlock(
    int startIndex
)
{
    double meanX = 0.0;
    double meanY = 0.0;
    double meanZ = 0.0;


    // --------------------------------------------------------
    // Calculate local DC
    // --------------------------------------------------------

    for (
        int i = startIndex;
        i <
        startIndex + ENV_BLOCK_SIZE;
        i++
    )
    {
        meanX += axBuffer[i];
        meanY += ayBuffer[i];
        meanZ += azBuffer[i];
    }


    meanX /=
        ENV_BLOCK_SIZE;


    meanY /=
        ENV_BLOCK_SIZE;


    meanZ /=
        ENV_BLOCK_SIZE;


    // --------------------------------------------------------
    // Calculate AC energy
    // --------------------------------------------------------

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;


    for (
        int i = startIndex;
        i <
        startIndex + ENV_BLOCK_SIZE;
        i++
    )
    {
        double x =
            axBuffer[i]
            -
            meanX;


        double y =
            ayBuffer[i]
            -
            meanY;


        double z =
            azBuffer[i]
            -
            meanZ;


        sumX +=
            x * x;


        sumY +=
            y * y;


        sumZ +=
            z * z;
    }


    float rmsX =
        sqrt(
            sumX /
            ENV_BLOCK_SIZE
        );


    float rmsY =
        sqrt(
            sumY /
            ENV_BLOCK_SIZE
        );


    float rmsZ =
        sqrt(
            sumZ /
            ENV_BLOCK_SIZE
        );


    // --------------------------------------------------------
    // Use strongest axis
    // --------------------------------------------------------

    return max(
        rmsX,
        max(
            rmsY,
            rmsZ
        )
    );
}


// ============================================================
// Vibration state + start/stop detection
// ============================================================

void processEnvelope(
    uint32_t captureStartUs,
    float fs
)
{
    for (
        int block = 0;
        block < ENV_BLOCK_COUNT;
        block++
    )
    {
        int index =
            block
            *
            ENV_BLOCK_SIZE;


        float envelope =
            calculateEnvelopeBlock(
                index
            );


        bool oldState =
            vibrationState;


        // ----------------------------------------------------
        // Hysteresis
        // ----------------------------------------------------

        if (!vibrationState)
        {
            if (
                envelope >=
                VIB_ON_THRESHOLD
            )
            {
                vibrationState =
                    true;
            }
        }
        else
        {
            if (
                envelope <=
                VIB_OFF_THRESHOLD
            )
            {
                vibrationState =
                    false;
            }
        }


        // ----------------------------------------------------
        // OFF -> ON rising edge
        // ----------------------------------------------------

        if (
            !oldState
            &&
            vibrationState
        )
        {
            float offsetSeconds =
                (float)index
                /
                fs;


            uint32_t edgeMs =
                (
                    captureStartUs
                    /
                    1000UL
                )
                +
                (uint32_t)(
                    offsetSeconds
                    *
                    1000.0f
                );


            previousRisingEdgeMs =
                lastRisingEdgeMs;


            lastRisingEdgeMs =
                edgeMs;


            if (
                previousRisingEdgeMs
                != 0
            )
            {
                uint32_t periodMs =
                    lastRisingEdgeMs
                    -
                    previousRisingEdgeMs;


                if (periodMs > 0)
                {
                    float frequency =
                        1000.0f
                        /
                        periodMs;


                    if (
                        frequency <=
                        STARTSTOP_MAX_HZ
                    )
                    {
                        startStopFrequency =
                            frequency;
                    }
                }
            }
        }
    }


    // --------------------------------------------------------
    // No new rising edge for 3 seconds:
    // treat as no start/stop periodicity.
    // --------------------------------------------------------

    if (
        lastRisingEdgeMs != 0
    )
    {
        if (
            millis()
            -
            lastRisingEdgeMs
            >
            STARTSTOP_TIMEOUT_MS
        )
        {
            startStopFrequency =
                0.0f;
        }
    }
}


// ============================================================
// Heartbeat LED
// ============================================================

void updateHeartbeat()
{
    static uint32_t lastTime =
        0;


    static bool state =
        false;


    if (
        millis()
        -
        lastTime
        >=
        500
    )
    {
        lastTime =
            millis();


        state =
            !state;


        digitalWrite(
            GREEN_LED_PIN,
            state
        );
    }
}


// ============================================================
// One complete measurement
// ============================================================

void runMeasurement()
{
    int i2cErrors =
        0;


    uint32_t firstSampleUs =
        0;


    uint32_t lastSampleUs =
        0;


    // ========================================================
    // Acquire 1024 samples
    // ========================================================

    float fs =
        captureSamples(
            i2cErrors,
            firstSampleUs,
            lastSampleUs
        );


    if (
        fs <= 0.0f
        ||
        i2cErrors > 0
    )
    {
        Serial.println(
            "ERROR=I2C"
        );

        return;
    }


    // ========================================================
    //Analyze short-time vibration envelope
    // ========================================================

    processEnvelope(
        firstSampleUs,
        fs
    );


    // ========================================================
    // FFT X axis
    // ========================================================

    FFTResult resultX =
        analyzeAxis(
            'X',
            axBuffer,
            fs
        );


    yield();


    // ========================================================
    //FFT Y axis
    // ========================================================

    FFTResult resultY =
        analyzeAxis(
            'Y',
            ayBuffer,
            fs
        );


    yield();


    // ========================================================
    //FFT Z axis
    // ========================================================

    FFTResult resultZ =
        analyzeAxis(
            'Z',
            azBuffer,
            fs
        );


    yield();


    // ========================================================
    // Select best spectral axis
    //
    // Select by spectral Peak Ratio,
    // NOT simply by RMS.
    // ========================================================

    FFTResult best =
        resultX;


    if (
        resultY.peakRatio
        >
        best.peakRatio
    )
    {
        best =
            resultY;
    }


    if (
        resultZ.peakRatio
        >
        best.peakRatio
    )
    {
        best =
            resultZ;
    }


    // ========================================================
    //Determine whether frequency is valid
    // ========================================================

    bool spectrumValid =
        (
            best.peakRatio
            >=
            MIN_PEAK_RATIO
        );


    // 正常频率：80~180 Hz
    bool frequencyInRange =
        (
            vibrationState
            &&
            spectrumValid
            &&
            best.peakFrequency
            >=
            FREQ_MIN
            &&
            best.peakFrequency
            <=
            FREQ_MAX
        );


    // 超过180 Hz时黄灯亮；180 Hz本身仍属于正常范围
    bool overFrequency =
        (
            vibrationState
            &&
            spectrumValid
            &&
            best.peakFrequency
            >
            FREQ_MAX
        );


    if (
        frequencyInRange
    )
    {
        mainFrequency =
            best.peakFrequency;
    }
    else
    {
        mainFrequency =
            0.0f;
    }


    // ========================================================
    // Yellow LED
    //
    // ON  = reliable vibration frequency > 180 Hz
    // OFF = frequency <= 180 Hz or no valid vibration
    // ========================================================

    digitalWrite(
        YELLOW_LED_PIN,
        overFrequency
        ?
        HIGH
        :
        LOW
    );


    // ========================================================
    // Compact UART output
    // ========================================================

    Serial.print(
        "VIB="
    );


    Serial.print(
        vibrationState
        ?
        1
        :
        0
    );


    Serial.print(
        ",FREQ="
    );


    Serial.print(
        mainFrequency,
        2
    );


    Serial.print(
        "Hz"
    );


    Serial.print(
        ",STARTSTOP="
    );


    Serial.print(
        startStopFrequency,
        3
    );


    Serial.print(
        "Hz"
    );


    Serial.print(
        ",RATIO="
    );


    Serial.println(
        best.peakRatio,
        2
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        SERIAL_BAUD
    );


    delay(1200);


    // --------------------------------------------------------
    // LEDs
    // --------------------------------------------------------

    pinMode(
        GREEN_LED_PIN,
        OUTPUT
    );


    pinMode(
        YELLOW_LED_PIN,
        OUTPUT
    );


    digitalWrite(
        GREEN_LED_PIN,
        LOW
    );


    digitalWrite(
        YELLOW_LED_PIN,
        LOW
    );


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );


    Wire.setClock(
        400000
    );


    delay(100);


    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------

    if (!initMPU6050())
    {
        Serial.println(
            "ERROR=MPU6050_INIT"
        );


        while (1)
        {
            digitalWrite(
                YELLOW_LED_PIN,
                !digitalRead(
                    YELLOW_LED_PIN
                )
            );


            delay(200);
        }
    }

    Serial.println(
        "SYSTEM=READY"
    );


    delay(500);
}


// ============================================================
// LOOP
// ============================================================
void loop()
{
    updateHeartbeat();


    runMeasurement();


    updateHeartbeat();


    delay(20);
}