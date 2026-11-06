#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================
//     正常振动频率: 80 ~ 180 Hz；FFT仍观察到300 Hz用于超频判断
//
// Current UART output:
//     VIB=1,FREQ=235.31Hz,STARTSTOP=0.000Hz,RATIO=69.19（RATIO是FFT谱峰相对背景的强度，>10数据可信度高）
//
// VIB     : 本次采集(约1秒)内出现过振动
// FREQ    : 本次采集测到的载波频率，80~180 Hz 之外显示 0
// STARTSTOP: 启停频率，取最近约8秒内上升沿的平均间隔，超过8 Hz按8 Hz输出，
//            超过5秒不再出现上升沿时归0
// RATIO   : 谱峰/背景
//
// Hardware:
//     ESP32-C3
//     MPU6050
// ============================================================

#define SDA_PIN                 10
#define SCL_PIN                 9

#define MPU_ADDR                0x68

#define SERIAL_BAUD             115200


// LED pins
#define GREEN_LED_PIN           6
#define YELLOW_LED_PIN          5


// UART mirror output (same data as the USB Type-C port)
//
// GPIO20 is used as TX only, so RX is left unassigned.
#define MIRROR_TX_PIN           20
#define MIRROR_BAUD             SERIAL_BAUD


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
//
// The 32 ms RMS window is kept, because the 120 / 180 levels
// were calibrated against it. The window now advances by
// ENV_BLOCK_HOP samples instead of a full window, so the
// envelope is evaluated every 8 ms: short start/stop pauses
// become visible and the rising-edge timestamp resolution drops
// from 32 ms to 8 ms.
// ============================================================

#define ENV_BLOCK_SIZE          32
#define ENV_BLOCK_HOP           8

#define ENV_BLOCK_COUNT         \
    (((SAMPLE_COUNT - ENV_BLOCK_SIZE) / ENV_BLOCK_HOP) + 1)

// Monitor burst used while the FFTs run. The three 1024 point
// FFTs take roughly 300 ms, and nothing was sampled during that
// time; a start/stop pause falling into that hole was lost. On
// the real machine this showed up as 3 rising edges per capture
// instead of 4 (1.335 s capture period, 3 start/stop) and a
// start/stop reading about 25% low.
//
// Each FFT is also split in the middle (see FFT_BURST_STAGE_LEN),
// which halves the largest unsampled hole from ~93 ms to ~46 ms.
#define MON_BURST_SAMPLES       50

#define FFT_BURST_STAGE_LEN     32

#define MON_BURST_WINDOWS       \
    (((MON_BURST_SAMPLES - ENV_BLOCK_SIZE) / ENV_BLOCK_HOP) + 1)


// ============================================================
// Vibration detection thresholds
//
// Based on current measurements:
//
// Bench machine (173 Hz carrier, 3 Hz start/stop):
//     quiet / pause level   approximately 500
//     running burst level   approximately 12000
//
// Older setup:
//     quiet 50~100, strong vibration 270~330
//
// Hysteresis:
//
// Both thresholds follow the running envelope peak, so the gap
// between them can never invert:
//
//     ON  threshold = max(180, 55% of the running peak)
//     OFF threshold = max(120, 28% of the running peak)
//
// With a peak of 12300 (the machine on the test bench) that is
// ON 6770 / OFF 3440, and with a quiet level of ~500 the pause is
// always unambiguous. The arm level used to be 75% of the peak,
// which the spin-up of some start/stop cycles did not sustain for
// long enough, so those pauses were dropped.
//
// With a peak of ~350 that is ON 193 / OFF 120. An earlier
// version kept the ON level fixed at 180 while raising the OFF
// level to 55% of the peak (~190 on the real machine), which put
// OFF *above* ON: the envelope then chattered across that band
// and produced dozens of fake rising edges per second.
//
// A fixed OFF threshold of 120 is what used to latch the
// detector ON: during a short start/stop pause the machine had
// not coasted down to 120 yet, so the state never reset, no
// further rising edge was ever seen and the reported start/stop
// rate was forced to 0 by the stale timeout. The relative
// threshold follows the actual level of the machine instead.
// ============================================================

#define VIB_ON_THRESHOLD        180.0f
#define VIB_ON_RATIO            0.55f

#define VIB_OFF_FLOOR           120.0f
#define VIB_OFF_RATIO           0.28f

// Decay of the running envelope peak, applied once per envelope
// block. 0.995 per 8 ms is roughly a 1.6 s time constant.
#define ENV_PEAK_DECAY          0.995f

// A capture counts as vibrating when at least this many envelope
// blocks exceed VIB_ON_THRESHOLD.
// 6 blocks * 8 ms hop = 48 ms of real vibration.
#define VIB_MIN_ON_BLOCKS       6

// A level crossing only counts after this many consecutive
// blocks, so threshold chatter cannot create fake edges.
// 3 blocks * 8 ms = 24 ms.
#define VIB_CONFIRM_BLOCKS      3


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

// ============================================================
// Start/stop rate estimation
//
// The rate is averaged over the rising edges of the last
// STARTSTOP_AVG_WINDOW_MS, so one missed edge only biases the
// value instead of doubling or halving it. Slow cycling falls
// back to the plain period between the last two edges.
//
// The bench machine (3 Hz) alternates short (~230 ms) and long
// (~460 ms) cycles, so a two or three cycle average wobbles
// between 2.2 and 2.9 Hz. Averaging over the whole window
// (about 7 s, 19 intervals at 3 Hz) settles it at 2.9 Hz, which
// matches the setpoint.
// ============================================================

// Reporting clamp only; it no longer discards measurements.
#define STARTSTOP_MAX_HZ        8.0f

#define STARTSTOP_EDGE_HISTORY  20
#define STARTSTOP_AVG_WINDOW_MS 8000
#define STARTSTOP_MIN_EDGES     4
#define STARTSTOP_SLOW_SPAN_MS  20000
#define STARTSTOP_HOLD_MS       5000


// ============================================================
// MPU6050 registers
// ============================================================
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C

#define REG_ACCEL_XOUT_H        0x3B

#define REG_PWR_MGMT_1          0x6B
#define REG_PWR_MGMT_2          0x6C
#define REG_WHO_AM_I            0x75

// ============================================================
// Sample buffers
// ============================================================
int16_t axBuffer[SAMPLE_COUNT];
int16_t ayBuffer[SAMPLE_COUNT];
int16_t azBuffer[SAMPLE_COUNT];

// Monitor burst buffer, sampled between the FFTs
int16_t monBufferX[MON_BURST_SAMPLES];
int16_t monBufferY[MON_BURST_SAMPLES];
int16_t monBufferZ[MON_BURST_SAMPLES];

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
// Instantaneous hysteresis state, drives rising-edge detection
bool vibrationState = false;
// Vibration anywhere inside the current capture
bool vibrationPresent = false;
// Running envelope peak with slow decay; reference level for the
// relative OFF threshold
float envelopePeak = 0.0f;
// Envelope sample source: the capture buffer or a monitor burst
int16_t *envBufX = axBuffer;
int16_t *envBufY = ayBuffer;
int16_t *envBufZ = azBuffer;
int envWindowCount = 0;
// Confirmation runs persist across captures and bursts, so a run
// may continue over a sampling gap.
int armRunBlocks = 0;
int offRunBlocks = 0;
uint32_t armRunStartMs = 0;
// Accumulators over one whole measurement (capture + bursts)
int measurementBlocks = 0;
// Current detected carrier frequency
float mainFrequency = 0.0f;
// Current start/stop frequency
float startStopFrequency = 0.0f;
// Rising-edge history, ring buffer of absolute times in ms
uint32_t lastRisingEdgeMs = 0;
uint32_t edgeTimeMs[STARTSTOP_EDGE_HISTORY];
int edgeWriteIndex = 0;
int edgeStoredCount = 0;

// ============================================================
// 主循环依赖的函数
// ============================================================

// 更新状态指示灯的心跳状态，且不会阻塞传感器采样。
// 主循环在每次测量前后调用此函数，使运行状态指示保持及时。
void updateHeartbeat();

// 执行一轮完整的振动测量：采集加速度样本，更新振动包络及启停状态，
// 分析频谱，更新状态指示灯，并将格式化结果发送到两个串口输出。
void runMeasurement();

// ============================================================
// 主循环
// ============================================================

// Arduino 在 setup() 执行完成后会反复调用 loop()。
// 每轮先更新心跳，再执行一次测量，随后再次更新心跳，
// 最后短暂延时，让出处理时间后进入下一轮。
void loop()
{
    updateHeartbeat();


    runMeasurement();


    updateHeartbeat();


    delay(20);
}

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
    // Wake MPU6050 using its internal oscillator.
    // --------------------------------------------------------

    if (
        !writeRegister(
            REG_PWR_MGMT_1,
            0x00
        )
    )
    {
        return false;
    }


    delay(100);

    // The accelerometer is used, but all three gyroscope axes are not.
    // STBY_XG/YG/ZG = 1; accelerometer standby bits remain clear.
    if (
        !writeRegister(
            REG_PWR_MGMT_2,
            0x07
        )
    )
    {
        return false;
    }


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

// The FFT calls this in the middle of its stage loop, so the
// envelope detector keeps sampling while the spectrum is being
// computed.
void captureMonitorBurst();


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


        // ----------------------------------------------------
        // Keep the envelope detector alive while the FFT runs.
        // ----------------------------------------------------

        if (len == FFT_BURST_STAGE_LEN)
        {
            captureMonitorBurst();
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
// AC variance from integer sums: E[x^2] - E[x]^2
// ============================================================

double acVariance(
    int64_t squareSum,
    int64_t sum,
    double count
)
{
    double mean =
        (double)sum
        /
        count;


    return
        (double)squareSum
        /
        count
        -
        mean
        *
        mean;
}


// ============================================================
// Calculate vibration envelope for one 32 ms window
//
// The sums are accumulated as integers, which is cheap on this
// FPU-less RISC-V core, and the AC RMS is then
//
//     sqrt( sum(x^2) / N - mean^2 )
//
// That is the same statistic as before, and it keeps the now
// 125 windows per capture practically free.
//
// The samples are read through the envBuf* pointers, so the same
// code serves the capture buffer and the monitor bursts.
// ============================================================

float calculateEnvelopeBlock(
    int startIndex
)
{
    int64_t sumX = 0;
    int64_t sumY = 0;
    int64_t sumZ = 0;

    int64_t squareX = 0;
    int64_t squareY = 0;
    int64_t squareZ = 0;


    // --------------------------------------------------------
    // Local sums (DC and energy)
    // --------------------------------------------------------

    for (
        int i = startIndex;
        i <
        startIndex + ENV_BLOCK_SIZE;
        i++
    )
    {
        int32_t x =
            envBufX[i];

        int32_t y =
            envBufY[i];

        int32_t z =
            envBufZ[i];


        sumX +=
            x;

        sumY +=
            y;

        sumZ +=
            z;


        squareX +=
            (int64_t)x * x;

        squareY +=
            (int64_t)y * y;

        squareZ +=
            (int64_t)z * z;
    }


    double count =
        (double)ENV_BLOCK_SIZE;


    double varianceX =
        acVariance(
            squareX,
            sumX,
            count
        );


    double varianceY =
        acVariance(
            squareY,
            sumY,
            count
        );


    double varianceZ =
        acVariance(
            squareZ,
            sumZ,
            count
        );


    // --------------------------------------------------------
    // AC RMS per axis (variance is clamped at 0, the integer
    // form can round a hair negative on a flat signal)
    // --------------------------------------------------------

    float rmsX =
        (float)sqrt(
            varianceX > 0.0
            ?
            varianceX
            :
            0.0
        );


    float rmsY =
        (float)sqrt(
            varianceY > 0.0
            ?
            varianceY
            :
            0.0
        );


    float rmsZ =
        (float)sqrt(
            varianceZ > 0.0
            ?
            varianceZ
            :
            0.0
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
// Record one rising edge in the history ring
// ============================================================

void recordRisingEdge(
    uint32_t edgeMs
)
{
    edgeTimeMs[edgeWriteIndex] =
        edgeMs;


    edgeWriteIndex =
        (edgeWriteIndex + 1)
        %
        STARTSTOP_EDGE_HISTORY;


    if (
        edgeStoredCount <
        STARTSTOP_EDGE_HISTORY
    )
    {
        edgeStoredCount++;
    }


    lastRisingEdgeMs =
        edgeMs;
}


// ============================================================
// Start/stop rate from the rising-edge history
//
// 1) Fast cycling: average over the edges inside the
//    observation window. One missed edge only biases the value
//    instead of doubling or halving it.
// 2) Slow cycling: single period between the last two edges.
// 3) No edge for STARTSTOP_HOLD_MS: report 0. Below that the
//    previous value is kept, so a single missed edge does not
//    blank the field.
// ============================================================

void updateStartStopFrequency(
    uint32_t nowMs
)
{
    int count = 0;

    uint32_t firstMs = 0;
    uint32_t lastMs = 0;


    for (
        int i = 0;
        i < edgeStoredCount;
        i++
    )
    {
        int slot =
            (
                edgeWriteIndex
                -
                edgeStoredCount
                +
                i
                +
                STARTSTOP_EDGE_HISTORY
            )
            %
            STARTSTOP_EDGE_HISTORY;


        uint32_t edgeMs =
            edgeTimeMs[slot];


        if (
            (uint32_t)(
                nowMs
                -
                edgeMs
            )
            >
            STARTSTOP_AVG_WINDOW_MS
        )
        {
            continue;
        }


        if (count == 0)
        {
            firstMs =
                edgeMs;
        }


        lastMs =
            edgeMs;


        count++;
    }


    // --------------------------------------------------------
    // 1) average rate over the observation window
    // --------------------------------------------------------

    if (
        count >=
        STARTSTOP_MIN_EDGES
        &&
        lastMs > firstMs
    )
    {
        float rate =
            (float)(
                count
                -
                1
            )
            *
            1000.0f
            /
            (float)(
                lastMs
                -
                firstMs
            );


        if (
            rate >
            STARTSTOP_MAX_HZ
        )
        {
            rate =
                STARTSTOP_MAX_HZ;
        }


        startStopFrequency =
            rate;


        return;
    }


    // --------------------------------------------------------
    // 2) slow start/stop: period between the last two edges
    // --------------------------------------------------------

    if (
        edgeStoredCount >= 2
        &&
        lastRisingEdgeMs != 0
        &&
        (uint32_t)(
            nowMs
            -
            lastRisingEdgeMs
        )
        <=
        STARTSTOP_HOLD_MS
    )
    {
        int previousSlot =
            (
                edgeWriteIndex
                -
                2
                +
                STARTSTOP_EDGE_HISTORY
            )
            %
            STARTSTOP_EDGE_HISTORY;


        uint32_t periodMs =
            lastRisingEdgeMs
            -
            edgeTimeMs[previousSlot];


        if (
            periodMs > 0
            &&
            periodMs <=
            STARTSTOP_SLOW_SPAN_MS
        )
        {
            float rate =
                1000.0f
                /
                (float)periodMs;


            if (
                rate >
                STARTSTOP_MAX_HZ
            )
            {
                rate =
                    STARTSTOP_MAX_HZ;
            }


            startStopFrequency =
                rate;


            return;
        }
    }


    // --------------------------------------------------------
    // 3) nothing periodic any more
    // --------------------------------------------------------

    if (
        lastRisingEdgeMs == 0
        ||
        (uint32_t)(
            nowMs
            -
            lastRisingEdgeMs
        )
        >
        STARTSTOP_HOLD_MS
    )
    {
        startStopFrequency =
            0.0f;
    }
}


// ============================================================
// Vibration presence + rising-edge detection
//
// Every sampled window is fed through here, whether it comes from
// the capture or from a monitor burst, so a start/stop pause can
// no longer hide in the time the FFTs need. Whether the
// measurement counts as vibrating is evaluated afterwards from
// measurementBlocks.
// ============================================================

void setEnvelopeSource(
    int16_t *x,
    int16_t *y,
    int16_t *z,
    int windowCount
)
{
    envBufX =
        x;

    envBufY =
        y;

    envBufZ =
        z;

    envWindowCount =
        windowCount;
}


void processEnvelope(
    uint32_t captureStartUs,
    float fs
)
{
    for (
        int block = 0;
        block < envWindowCount;
        block++
    )
    {
        int index =
            block
            *
            ENV_BLOCK_HOP;


        uint32_t blockMs =
            (
                captureStartUs
                /
                1000UL
            )
            +
            (uint32_t)(
                (float)index
                /
                fs
                *
                1000.0f
            );


        float envelope =
            calculateEnvelopeBlock(
                index
            );


        // ----------------------------------------------------
        // Running peak and hysteresis levels
        // ----------------------------------------------------

        envelopePeak *=
            ENV_PEAK_DECAY;


        if (
            envelope >
            envelopePeak
        )
        {
            envelopePeak =
                envelope;
        }


        float onLevel =
            VIB_ON_RATIO
            *
            envelopePeak;


        if (
            onLevel <
            VIB_ON_THRESHOLD
        )
        {
            onLevel =
                VIB_ON_THRESHOLD;
        }


        float offLevel =
            VIB_OFF_RATIO
            *
            envelopePeak;


        if (
            offLevel <
            VIB_OFF_FLOOR
        )
        {
            offLevel =
                VIB_OFF_FLOOR;
        }


        if (
            envelope >=
            VIB_ON_THRESHOLD
        )
        {
            measurementBlocks++;
        }


        bool oldState =
            vibrationState;


        // ----------------------------------------------------
        // Hysteresis with level confirmation
        //
        // A crossing only takes effect after VIB_CONFIRM_BLOCKS
        // consecutive blocks beyond the level. Chatter around a
        // threshold therefore cannot fabricate rising edges.
        // ----------------------------------------------------

        if (!vibrationState)
        {
            if (
                envelope >=
                onLevel
            )
            {
                if (armRunBlocks == 0)
                {
                    armRunStartMs =
                        blockMs;
                }


                armRunBlocks++;


                if (
                    armRunBlocks >=
                    VIB_CONFIRM_BLOCKS
                )
                {
                    vibrationState =
                        true;


                    // Start the next run from scratch, otherwise
                    // the following edge would reuse the timestamp
                    // of this run's first block.
                    armRunBlocks =
                        0;
                }
            }
            else
            {
                armRunBlocks =
                    0;
            }
        }
        else
        {
            if (
                envelope <=
                offLevel
            )
            {
                offRunBlocks++;


                if (
                    offRunBlocks >=
                    VIB_CONFIRM_BLOCKS
                )
                {
                    vibrationState =
                        false;


                    offRunBlocks =
                        0;
                }
            }
            else
            {
                offRunBlocks =
                    0;
            }
        }


        // ----------------------------------------------------
        // OFF -> ON rising edge
        //
        // The timestamp is the first block of the confirmed run,
        // so the confirmation delay does not bias the measured
        // period.
        // ----------------------------------------------------

        if (
            !oldState
            &&
            vibrationState
        )
        {
            recordRisingEdge(
                armRunStartMs
            );
        }
    }


    // --------------------------------------------------------
    // Refresh the start/stop rate of this capture from the
    // rising-edge history.
    // --------------------------------------------------------

    updateStartStopFrequency(
        millis()
    );
}


// ============================================================
// Monitor burst
//
// Samples MON_BURST_SAMPLES at the normal 1 ms cadence and feeds
// them to the envelope detector. Called between the FFTs, so the
// detector keeps watching the machine during the analysis
// instead of losing every pause that happens to fall into that
// window.
// ============================================================

void captureMonitorBurst()
{
    uint32_t nextSampleTime =
        micros();


    uint32_t startUs =
        0;


    for (
        int i = 0;
        i < MON_BURST_SAMPLES;
        i++
    )
    {
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
            startUs =
                sampleTime;
        }


        if (
            !readAcceleration(
                monBufferX[i],
                monBufferY[i],
                monBufferZ[i]
            )
        )
        {
            // Same policy as the main capture: reuse the previous
            // sample instead of injecting a zero.

            if (i > 0)
            {
                monBufferX[i] =
                    monBufferX[i - 1];

                monBufferY[i] =
                    monBufferY[i - 1];

                monBufferZ[i] =
                    monBufferZ[i - 1];
            }
            else
            {
                monBufferX[i] = 0;
                monBufferY[i] = 0;
                monBufferZ[i] = 0;
            }
        }


        nextSampleTime +=
            SAMPLE_PERIOD_US;
    }


    setEnvelopeSource(
        monBufferX,
        monBufferY,
        monBufferZ,
        MON_BURST_WINDOWS
    );


    processEnvelope(
        startUs,
        1000.0f
    );
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
    //
    // The capture is scanned first, then a monitor burst after
    // each FFT, so no pause is lost while the spectrum is being
    // computed. "Vibration present" is decided at the end from
    // the accumulated blocks instead of the last block only, so a
    // capture ending in the quiet phase no longer reports 0 while
    // the spectrum is perfectly valid.
    // ========================================================

    measurementBlocks = 0;


    setEnvelopeSource(
        axBuffer,
        ayBuffer,
        azBuffer,
        ENV_BLOCK_COUNT
    );


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


    captureMonitorBurst();


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


    captureMonitorBurst();


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


    captureMonitorBurst();


    vibrationPresent =
        (
            measurementBlocks
            >=
            VIB_MIN_ON_BLOCKS
        );


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
            vibrationPresent
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
            vibrationPresent
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
    // ON  = vibration present in this capture and a reliable
    //       vibration frequency > 180 Hz
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
    //
    // The whole line is built once, then written to both the
    // USB Type-C port and the GPIO20 UART, so both carry the
    // exact same bytes at the exact same rate.
    // ========================================================

    char outputLine[96];


    snprintf(
        outputLine,
        sizeof(outputLine),
        "VIB=%d,FREQ=%.2fHz,STARTSTOP=%.3fHz,RATIO=%.2f",
        vibrationPresent
        ?
        1
        :
        0,
        mainFrequency,
        startStopFrequency,
        best.peakRatio
    );


    Serial.println(
        outputLine
    );


    Serial1.println(
        outputLine
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
    // Mirrored UART on GPIO20
    //
    // RX is not needed, so it is disabled with -1.
    // --------------------------------------------------------

    Serial1.begin(
        MIRROR_BAUD,
        SERIAL_8N1,
        -1,
        MIRROR_TX_PIN
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
