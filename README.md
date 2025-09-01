# esp32-hsp-hf  

オリジナルの [@atomic14/esp32-hsp-hf](https://github.com/atomic14/esp32-hsp-hf)  を、esp-idf v5.5 用に変更しました。  

ESP-EYE のマイクを、Bluetooth HSP で、PC につなげて音を確認することが目的なので、  
ESP-EYE の方には、スピーカーがないので、I2S write 部分は、ダミーです。  

##### 参照。  
詳しくは、下記ページをみとうせ!!  
[ESP-EYE マイク を bluetooth で、PC につなげて聞きたい。](https://www.netosa.com/blog/2025/08/esp-eye-bluetooth-pc.html)  
