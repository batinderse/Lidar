# Lidar Verisi ve Geometri Denemesi

C ile yazılmış bu örnek program, LIDAR tarama verilerini okuyup işleyerek ölçüm noktaları ve doğru kesişimleri üzerinde hesaplamalar yapar. Depoda örnek tarama verileri (`.dat` dosyaları ve `temp.toml`) bulunur.

## Gereksinimler

- C derleyicisi (ör. GCC)
- libcurl geliştirme kitaplığı
- Matematik kitaplığı

## Derleme ve çalıştırma

libcurl sistemde kurulu ve derleyici tarafından bulunabilir olmalıdır:

```bash
gcc -std=c11 main.c -o lidar -lcurl -lm
./lidar
```

Program çalışırken LIDAR girdisini `temp.toml` dosyasına indirir ve ardından dosyadaki açı, mesafe ve `ranges` değerlerini işler. Kodda kullanılan girdi indirme adresi `main.c` içinde tanımlıdır; ağ erişimi gerekir. Başarılı çalıştırmanın ardından terminal çıktısını ve oluşturulan `.dat` dosyalarını inceleyebilirsiniz.
