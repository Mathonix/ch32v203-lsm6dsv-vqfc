/**
 * Host float VQF-C vs vqf_fixed replay (design §18).
 * Build: make -C tools/vqf_fixed_replay
 *
 * Feeds the same synthetic (or CSV) gyro/acc stream to both filters at
 * gyr=4 kHz / acc=1 kHz and reports quat geodesic + Euler RMS error.
 */
#include "host_shim.h"

#include "vqf.h"
#include "vqf_fixed.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void quat_normalize4(double q[4])
{
    double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-18) {
        q[0] = 1.0;
        q[1] = q[2] = q[3] = 0.0;
        return;
    }
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}

static double quat_geodesic_deg(const double a[4], const double b[4])
{
    double d = fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    if (d > 1.0) {
        d = 1.0;
    }
    return 2.0 * acos(d) * (180.0 / M_PI);
}

static void quat_to_euler_deg(const double q[4], double *roll, double *pitch, double *yaw)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI);

    double sinp = 2.0 * (w * y - z * x);
    if (sinp > 1.0) {
        sinp = 1.0;
    } else if (sinp < -1.0) {
        sinp = -1.0;
    }
    *pitch = asin(sinp) * (180.0 / M_PI);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);
}

static void f30_to_unit(const int32_t qf[4], double q[4])
{
    const double s = 1.0 / (double)(1 << 30);
    q[0] = (double)qf[0] * s;
    q[1] = (double)qf[1] * s;
    q[2] = (double)qf[2] * s;
    q[3] = (double)qf[3] * s;
    quat_normalize4(q);
}

typedef struct {
    double gyr[3]; /* rad/s */
    double acc[3]; /* m/s² */
} sample_t;

/** Synthetic: gravity + slow yaw rate + small noise; 4 kHz gyro, 1 kHz acc. */
static void gen_synthetic(sample_t *gyr_s, sample_t *acc_s, int n_gyr)
{
    const double g = 9.80665;
    const double yaw_rate = 5.0 * M_PI / 180.0; /* 5 deg/s */
    for (int i = 0; i < n_gyr; i++) {
        double t = (double)i * 0.00025;
        gyr_s[i].gyr[0] = 0.01 * sin(2.0 * M_PI * 0.3 * t);
        gyr_s[i].gyr[1] = 0.01 * cos(2.0 * M_PI * 0.2 * t);
        gyr_s[i].gyr[2] = yaw_rate + 0.002 * sin(2.0 * M_PI * 0.5 * t);
        /* Acc held piecewise-constant per 4 gyro ticks (1 kHz). */
        int ai = i / 4;
        double ta = (double)ai * 0.001;
        /* Mild pitch oscillation so inclination path is exercised */
        double pitch = 0.05 * sin(2.0 * M_PI * 0.1 * ta); /* rad */
        acc_s[i].acc[0] = g * sin(pitch);
        acc_s[i].acc[1] = 0.0;
        acc_s[i].acc[2] = g * cos(pitch);
        gyr_s[i].acc[0] = acc_s[i].acc[0];
        gyr_s[i].acc[1] = acc_s[i].acc[1];
        gyr_s[i].acc[2] = acc_s[i].acc[2];
    }
}

static int load_csv(const char *path, sample_t **out, int *n_out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    int cap = 4096;
    int n = 0;
    sample_t *buf = (sample_t *)malloc((size_t)cap * sizeof(sample_t));
    if (!buf) {
        fclose(f);
        return -2;
    }
    char line[256];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        double t, gx, gy, gz, ax, ay, az;
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &t, &gx, &gy, &gz, &ax, &ay, &az) != 7) {
            continue;
        }
        if (n >= cap) {
            cap *= 2;
            sample_t *nb = (sample_t *)realloc(buf, (size_t)cap * sizeof(sample_t));
            if (!nb) {
                free(buf);
                fclose(f);
                return -3;
            }
            buf = nb;
        }
        buf[n].gyr[0] = gx;
        buf[n].gyr[1] = gy;
        buf[n].gyr[2] = gz;
        buf[n].acc[0] = ax;
        buf[n].acc[1] = ay;
        buf[n].acc[2] = az;
        n++;
    }
    fclose(f);
    *out = buf;
    *n_out = n;
    return 0;
}

int main(int argc, char **argv)
{
    const char *csv = NULL;
    int seconds = 10;
    int use_static = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--csv") && i + 1 < argc) {
            csv = argv[++i];
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = atoi(argv[++i]);
            if (seconds < 1) {
                seconds = 1;
            }
        } else if (!strcmp(argv[i], "--static")) {
            use_static = 1;
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr,
                    "usage: %s [--seconds N] [--static] [--csv file]\n"
                    "  CSV columns: t,gx,gy,gz,ax,ay,az  (gyr rad/s, acc m/s^2)\n"
                    "  Default: synthetic mild motion @ 4 kHz gyr / 1 kHz acc\n"
                    "  --static: zero gyro + +1g Z (should be near bit-exact)\n",
                    argv[0]);
            return 0;
        }
    }

    sample_t *stream = NULL;
    int n_gyr = 0;
    if (csv) {
        if (load_csv(csv, &stream, &n_gyr) != 0 || n_gyr < 8) {
            fprintf(stderr, "failed to load CSV %s\n", csv);
            return 1;
        }
        printf("Loaded %d samples from %s\n", n_gyr, csv);
    } else {
        n_gyr = seconds * 4000;
        stream = (sample_t *)calloc((size_t)n_gyr, sizeof(sample_t));
        if (!stream) {
            return 1;
        }
        if (use_static) {
            for (int i = 0; i < n_gyr; i++) {
                stream[i].gyr[0] = stream[i].gyr[1] = stream[i].gyr[2] = 0.0;
                stream[i].acc[0] = 0.0;
                stream[i].acc[1] = 0.0;
                stream[i].acc[2] = 9.80665;
            }
            printf("Static stream: %d gyro samples (%.1f s), zero gyr + +1g\n", n_gyr,
                   (double)seconds);
        } else {
            gen_synthetic(stream, stream, n_gyr);
            printf("Synthetic motion: %d gyro samples (%.1f s @ 4 kHz), acc every 4th\n",
                   n_gyr, (double)seconds);
        }
    }

    /* Init both filters at design rates */
    initVqf(0.00025f, 0.001f, 0.01f);
    vqf_fixed_config_t cfg = {.gyr_hz = 4000u, .acc_hz = 1000u, .mag_hz = 100u};
    vqf_fixed_init(&cfg);

    double sum_ang2 = 0.0, sum_r2 = 0.0, sum_p2 = 0.0, sum_y2 = 0.0;
    double max_ang = 0.0;
    int n_cmp = 0;
    /* p99 approx: keep simple reservoir of last errors sorted — or just track max + RMS */

    for (int i = 0; i < n_gyr; i++) {
        float gyr_f[3] = {
            (float)stream[i].gyr[0],
            (float)stream[i].gyr[1],
            (float)stream[i].gyr[2],
        };
        updateGyr(gyr_f);

        int32_t g25[3];
        for (int k = 0; k < 3; k++) {
            double v = stream[i].gyr[k] * (double)(1 << 25);
            if (v > 2147483647.0) {
                v = 2147483647.0;
            }
            if (v < -2147483648.0) {
                v = -2147483648.0;
            }
            g25[k] = (int32_t)lrint(v);
        }
        vqf_fixed_update_gyr_f25(g25);

        if ((i % 4) == 0) {
            float acc_f[3] = {
                (float)stream[i].acc[0],
                (float)stream[i].acc[1],
                (float)stream[i].acc[2],
            };
            updateAcc(acc_f);

            int32_t a27[3];
            for (int k = 0; k < 3; k++) {
                double g_unit = stream[i].acc[k] / 9.80665;
                double v = g_unit * (double)(1 << 27);
                if (v > 2147483647.0) {
                    v = 2147483647.0;
                }
                if (v < -2147483648.0) {
                    v = -2147483648.0;
                }
                a27[k] = (int32_t)lrint(v);
            }
            vqf_fixed_update_acc_f27(a27);

            vqf_real_t qf[4];
            getQuat6D(qf);
            double qfd[4] = {qf[0], qf[1], qf[2], qf[3]};
            quat_normalize4(qfd);

            int32_t qx[4];
            vqf_fixed_get_quat6d_f30(qx);
            double qxd[4];
            f30_to_unit(qx, qxd);

            double ang = quat_geodesic_deg(qfd, qxd);
            double rf, pf, yf, rx, px, yx;
            quat_to_euler_deg(qfd, &rf, &pf, &yf);
            quat_to_euler_deg(qxd, &rx, &px, &yx);
            double er = rf - rx, ep = pf - px, ey = yf - yx;
            /* wrap yaw diff roughly */
            if (ey > 180.0) {
                ey -= 360.0;
            }
            if (ey < -180.0) {
                ey += 360.0;
            }

            sum_ang2 += ang * ang;
            sum_r2 += er * er;
            sum_p2 += ep * ep;
            sum_y2 += ey * ey;
            if (ang > max_ang) {
                max_ang = ang;
            }
            n_cmp++;
        }
    }

    if (n_cmp < 1) {
        fprintf(stderr, "no comparison samples\n");
        free(stream);
        return 1;
    }

    double rms_ang = sqrt(sum_ang2 / n_cmp);
    double rms_r = sqrt(sum_r2 / n_cmp);
    double rms_p = sqrt(sum_p2 / n_cmp);
    double rms_y = sqrt(sum_y2 / n_cmp);

    printf("Compared %d attitude samples (acc rate)\n", n_cmp);
    printf("Quat geodesic RMS: %.6f deg  (max %.6f deg)\n", rms_ang, max_ang);
    printf("Euler RMS deg:     roll=%.6f pitch=%.6f yaw=%.6f\n", rms_r, rms_p, rms_y);
    printf("Design target:     RMS < 0.02 deg (engineering)\n");
    if (rms_ang < 0.02) {
        printf("RESULT: PASS (RMS < 0.02 deg)\n");
    } else {
        printf("RESULT: BELOW TARGET — Kalman/IIR fidelity still engineering-grade; see README\n");
    }

    free(stream);
    return 0;
}
