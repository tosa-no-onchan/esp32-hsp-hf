# esp32-hsp-hf  

オリジナルの [@atomic14/esp32-hsp-hf](https://github.com/atomic14/esp32-hsp-hf)  を、esp-idf v5.5 用に変更しました。  

ESP-EYE のマイクを、Bluetooth HSP で、PC につなげて音を確認することが目的なので、  
ESP-EYE の方には、スピーカーがないので、I2S write 部分は、ダミーです。  

##### 参照。  
詳しくは、下記ページをみとうせ!!  
[ESP-EYE マイク を bluetooth で、PC につなげて聞きたい。](https://www.netosa.com/blog/2025/08/esp-eye-bluetooth-pc.html)  


#### 注)  

 [@bluekitchen/btstack](https://github.com/bluekitchen/btstack)  がバージョンアップして、使い方が変わったみたい。  
 従来のやり方もできるようだが、integrate_btstack.py がなくなってしまった。  
 btstack-old-version/port/esp32/integrate_btstack.py に、従来の Python Script を上げておいたので、  
 従来のように使ってください。  

 esp-idf を v6.0 にしたら、Mic の音が、Host PC からでなくなった。  
 この場合、esp-idf v5.5 に戻して、sdkconfig.org-2025.8.29 を sdkconfig コピーして、再ビルドして試してください。  
