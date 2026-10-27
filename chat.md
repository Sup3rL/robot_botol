esp 1 fungsi
control 4 motor
control speed 4 motor 
control griper 

-------------------------------------
esp 2 fungsi 
menerima sinyal remot
lifter up down
lifter right left
encoder
sensor compas

komunikasi fungsi  
1. mengirimkan data remot griper dari esp2 ke esp1 
2. mengirimkan printah gerak(maju mundur kiri atau kanan) dari esp 2 ke esp1
3. mengirimkan printah speed 4 motor motor 

algoritma sensor dan remot 
1. esp menerima printrah remot 
2. encoder mengghitung eror roda 
3. kompas menghitung eror sudut 
hasil di kirim ke esp 1 dengan delay seminimal mungkin 