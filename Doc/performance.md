# RGIOE性能测试
# Performance of RGIOE
---
## 1\. FOG
IMU POS830:
ARW: 0.003, deg/s/sqrt(hr
VRW: 0.03, m/s/sqrt(hr)
Gyro bias stability: 0.027, deg/hr
Acce bias stability: 15, mGal
Gyro Scale factor std: 300, ppm
Acce Scale factor std: 300, ppm

| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.0071|0.0084|0.0079|
|position error(m) 2-$\sigma$|0.0187|0.0197|0.0162|
|position error(m) rms|0.0084|0.0109|0.0083|
|velocity error(m/s) 1-$\sigma$|0.0021|0.0018|0.0014|
|velocity error(m/s) 2-$\sigma$|0.0052|0.0047|0.0027|
|velocity error(m/s) rms|0.0105|0.0029|0.0014|
|attitude error(deg) 1-$\sigma$|0.0007|0.0008|0.0094|
|attitude error(deg) 2-$\sigma$|0.0016|0.0016|0.0161|
|attitude error(deg) rms|0.0158|0.0158|0.1520|
|2D error 1-$\sigma$|0.0126| | |
|2D error 2-$\sigma$|0.0237| | |
|2D error RMS\$|0.0137| | |
|3D error 1-$\sigma$|0.0154| | |
|3D error 2-$\sigma$|0.0261| | |
|3D error RMS $\sigma$|0.0160| | |

Outage 90s

| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.0751|0.0839|0.0626|
|position error(m) 2-$\sigma$|0.5434|0.7348|0.3131|
|position error(m) rms|0.2083|0.3328|0.1293|
|velocity error(m/s) 1-$\sigma$|0.0037|0.0040|0.0020|
|velocity error(m/s) 2-$\sigma$|0.0172|0.0133|0.0072|
|velocity error(m/s) rms|0.0131|0.0058|0.0030|
|attitude error(deg) 1-$\sigma$|0.0008|0.0010|0.0088|
|attitude error(deg) 2-$\sigma$|0.0015|0.0023|0.0257|
|attitude error(deg) rms|0.0096|0.0155|0.0202|
|2D error 1-$\sigma$|0.1757| | |
|2D error 2-$\sigma$|0.9696| | |
|2D error RMS\$|0.3926| | |
|3D error 1-$\sigma$|0.2175| | |
|3D error 2-$\sigma$|0.9848| | |
|3D error RMS $\sigma$|0.4134| | |
|Outage 2D|0.7719| | |
|Outage 3D|0.8624| | |
|Outage Pos|0.5133|0.8463|0.3492|
|Outage Atti|0.0006|0.0012|0.0104|

## 2\. MEMS1
IMU ADIS16465
ARW: 0.15, deg/s/sqrt(hr
VRW: 0.012, m/s/sqrt(hr)
Gyro bias stability: 2, deg/hr
Acce bias stability:  3.6, mGal
Gyro Scale factor std: 300, ppm
Acce Scale factor std: 300, ppm

| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.0217|0.0129|0.0399|
|position error(m) 2-$\sigma$|0.0347|0.0252|0.0900|
|position error(m) rms|0.0233|0.0128|0.0418|
|velocity error(m/s) 1-$\sigma$|0.0174|0.0169|0.0112|
|velocity error(m/s) 2-$\sigma$|0.0381|0.0399|0.0228|
|velocity error(m/s) rms|0.0194|0.0197|0.0113|
|attitude error(deg) 1-$\sigma$|0.0246|0.0186|0.3241|
|attitude error(deg) 2-$\sigma$|0.0481|0.0387|0.6254|
|attitude error(deg) rms|0.0246|0.0195|0.3407|
|2D error 1-$\sigma$|0.0256| | |
|2D error 2-$\sigma$|0.0371| | |
|2D error RMS\$|0.0266| | |
|3D error 1-$\sigma$|0.0468| | |
|3D error 2-$\sigma$|0.0943| | |
|3D error RMS $\sigma$|0.0496| | |

Outage 90s, with ZUPT NHC ODO 

| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.1381|0.0881|0.2381|
|position error(m) 2-$\sigma$|0.3874|0.1901|0.3524|
|position error(m) rms|0.1634|0.0925|0.2116|
|velocity error(m/s) 1-$\sigma$|0.0155|0.0159|0.0102|
|velocity error(m/s) 2-$\sigma$|0.0348|0.0369|0.0210|
|velocity error(m/s) rms|0.0176|0.0184|0.0104|
|attitude error(deg) 1-$\sigma$|0.0227|0.0176|0.1964|
|attitude error(deg) 2-$\sigma$|0.0430|0.0377|0.3594|
|attitude error(deg) rms|0.0221|0.0192|0.1901|
|2D error 1-$\sigma$|0.1768| | |
|2D error 2-$\sigma$|0.4120| | |
|2D error RMS\$|0.1878| | |
|3D error 1-$\sigma$|0.3095| | |
|3D error 2-$\sigma$|0.5073| | |
|3D error RMS $\sigma$|0.2829| | |
|Outage 2D|0.3015| | |
|Outage 3D|0.4818| | |
|Outage Pos|0.2969|0.1564|0.3564|
|Outage Atti|0.0247|0.0194|0.2192|

## GNSS Outage Test
test IMU: IMU ADIS16465
outage time: 60s
21D Kalman Filter

### 1\. NHC:OFF ODO:OFF
| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.3365|0.2356|0.1130|
|position error(m) 2-$\sigma$|6.9633|6.0847|1.0055|
|position error(m) rms|2.7915|2.5257|0.3854|
|velocity error(m/s) 1-$\sigma$|0.0413|0.0399|0.0151|
|velocity error(m/s) 2-$\sigma$|0.3566|0.3386|0.0342|
|velocity error(m/s) rms|0.1351|0.1380|0.0161|
|attitude error(deg) 1-$\sigma$|0.0323|0.0274|1.5935|
|attitude error(deg) 2-$\sigma$|0.0840|0.0805|4.9253|
|attitude error(deg) rms|0.0391|0.0347|2.0379|
|2D error 1-$\sigma$|0.5289| | |
|2D error 2-$\sigma$|9.5268| | |
|2D error RMS\$|3.7645| | |
|3D error 1-$\sigma$|0.5742| | |
|3D error 2-$\sigma$|9.5699| | |
|3D error RMS $\sigma$|3.7842| | |
|Outage 2D|12.6407| | |
|Outage 3D|12.7048| | |
|Outage Pos|10.4402|9.7360|1.1194|
|Outage Atti|0.0742|0.0668|2.5529|

|![image](images/NUNFznrvMtqV2iO-UNa24G5NMEwFGNAxUUDwACvDojE.png)<br>|![image](images/ahh3TKNuXt9WZFkE_OOZAM6oHgEjCuNZ1eJh9mTftvQ.png)<br>|
| ----- | ----- |

### 2\. NHC: ON ODO: OFF


| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.0629|0.1457|0.0569|
|position error(m) 2-$\sigma$|0.9779|1.8545|0.2209|
|position error(m) rms|0.3646|0.7456|0.0894|
|velocity error(m/s) 1-$\sigma$|0.0313|0.0209|0.0116|
|velocity error(m/s) 2-$\sigma$|0.1371|0.0699|0.0236|
|velocity error(m/s) rms|0.0553|0.0322|0.0118|
|attitude error(deg) 1-$\sigma$|0.0246|0.0234|0.1761|
|attitude error(deg) 2-$\sigma$|0.0475|0.0555|0.5667|
|attitude error(deg) rms|0.0242|0.0261|0.2464|
|2D error 1-$\sigma$|0.2843| | |
|2D error 2-$\sigma$|2.0493| | |
|2D error RMS\$|0.8300| | |
|3D error 1-$\sigma$|0.2982| | |
|3D error 2-$\sigma$|2.0574| | |
|3D error RMS $\sigma$|0.8348| | |
|Outage 2D|1.5721| | |
|Outage 3D|1.5791| | |
|Outage Pos|0.8261|1.7272|0.1584|
|Outage Atti|0.0181|0.0361|0.6156|



### 3\. NHC:ON ODO:ON
| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.0558|0.0431|0.1755|
|position error(m) 2-$\sigma$|0.5269|0.2749|0.4344|
|position error(m) rms|0.2364|0.2624|0.1913|
|velocity error(m/s) 1-$\sigma$|0.0182|0.0188|0.0134|
|velocity error(m/s) 2-$\sigma$|0.0456|0.0519|0.0397|
|velocity error(m/s) rms|0.0221|0.0250|0.0171|
|attitude error(deg) 1-$\sigma$|0.0229|0.0209|0.2235|
|attitude error(deg) 2-$\sigma$|0.0434|0.0522|1.3090|
|attitude error(deg) rms|0.0223|0.0239|0.7302|
|2D error 1-$\sigma$|0.0998| | |
|2D error 2-$\sigma$|0.6990| | |
|2D error RMS\$|0.3532| | |
|3D error 1-$\sigma$|0.2413| | |
|3D error 2-$\sigma$|0.7396| | |
|3D error RMS $\sigma$|0.4017| | |
|Outage 2D|0.7472| | |
|Outage 3D|0.8344| | |
|Outage Pos|0.6865|0.9630|0.2984|
|Outage Atti|0.0127|0.0253|1.8705|

