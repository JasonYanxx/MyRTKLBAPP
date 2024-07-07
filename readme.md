## How to use the code

### Download data

In the terminal, execute

```shell
rnx2rtkp\dataset\down_eph.sh # download ephemeris data
rnx2rtkp\dataset\download_obs.sh # download observation data
```

### Parameter setting

Main algorithm is implemented in `rnx2rtkp\src\pntpos.c`, specifically we add two function

```c++
static int pseudocor(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const double *vare, const int *svh, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *AziEle, int *vsat,
                  double *resp, char *msg, double *Praw, double *Pcor,double *Rtrue, double *measVar);

static void output_pseudocor(const char* output_filename, const obsd_t *obs, int n, double* rs, 
                            double* Praw, double* Pcor, double *Rtrue,
                            double *AziEle,double *measVar);
```

Below are parameters to be set in `pseudocor()`

- receiver position in ECEF (line 348) 

Once the parameter is re-set, one should re-build the code.

### Build code
- make sure `rnx2rtkp` is the root folder of vscode
- Press `Ctrl+Shift+B` with file `rnx2rtkp.c` open

### Execute code

- make sure `rnx2rtkp` is the root folder of vscode
- set `epoch` parameter in `rnx2rtkp\.vscode\settings.json`
- execute `rnx2rtkp\.vscode\tasks.json` in vscode:  press `F5` with file `rnx2rtkp.c` open

### Results

- `log{k}.csv` where `k` is associated with the `epoch` parameter in `rnx2rtkp\.vscode\settings.json`
- merge all log files with matlab code in [Matlab_PGO](https://github.com/JasonYanxx/Matlab_PGO)