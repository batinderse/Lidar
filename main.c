#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <curl/curl.h>

struct LidarData
{
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    float* ranges;
    size_t ranges_count;
} ;
struct Point
{
    double x;
    double y;
};
struct dogru_uc_noktalari
{
    double xx1;
    double yy1;
    double xx2;
    double yy2;
};
struct kesisim_nokta
{
    double xx;
    double yy;
    double acii;
};
size_t write_callback(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
    return fwrite(ptr, size, nmemb, stream);
}

int main()
{

    srand(time(NULL));
    const char* url = "https://drive.usercontent.google.com/download?id=1Phsyyc6N7bPFJT7XaHgNjYLdALgnqSPd&export=download&authuser=0";
    const char* filename = "temp.toml";
    struct LidarData lidar_verisi; // Pointer yerine doğrudan struct'ın kendisi
    struct dogru_uc_noktalari dogru[100];
    struct kesisim_nokta kesnokta[100];

    FILE *makine = fopen("makine.dat", "w");
    fprintf(makine, "0 0\n");
    fclose(makine);


    int kesnoktasayac=0;
    lidar_verisi.angle_min=0;
    lidar_verisi.range_max=0;
    lidar_verisi.angle_increment=0;
    lidar_verisi.range_min=0;
    lidar_verisi.range_max=0;
    lidar_verisi.ranges=NULL;
    lidar_verisi.ranges_count=0;
    char dosyaad[100];
    double aci_deg;

    CURL *curl = curl_easy_init();
    if (curl)
    {
        FILE *fp = fopen(filename, "wb");
        if (!fp)
        {
            printf("Dosya olusmadi..\n");
            curl_easy_cleanup(curl);
            return 1;
        }

        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

        CURLcode res = curl_easy_perform(curl);
        fclose(fp);

        if (res != CURLE_OK)
        {
            fprintf(stderr, "İndirme hatasi %s\n", curl_easy_strerror(res));
            curl_easy_cleanup(curl);
            return 1;
        }

        curl_easy_cleanup(curl);
        printf("Dosya basariyla indirildi %s\n", filename);
    }
    else
    {
        printf("Libcurl baslatilamadi..\n");
        return 1;
    }

    FILE* file = fopen(filename, "r");
    if (file == NULL)
    {
        printf("Dosya acilamadi.. %s\n", filename);
        return 1;
    }





    char buffer[512];
    int rangemod =0;


    while (fgets(buffer, sizeof(buffer), file))   // Dosyanın sonuna gelene kadar satır satır oku
    {
        if (rangemod)
        {
            // "]" karakterini gorunce bittigini anlar
            if (strstr(buffer, "]"))
            {
                rangemod=0;
                continue;
            }
            //range okuma modu
            char* token = strtok(buffer, ", \t\n"); //Satırı virgüle göre ayırdık
            while (token != NULL)
            {
                lidar_verisi.ranges_count++;
                lidar_verisi.ranges = (float*)realloc(lidar_verisi.ranges, lidar_verisi.ranges_count * sizeof(float));
                if (lidar_verisi.ranges == NULL)
                {
                    printf("Bellek ayirma basarisiz oldu...Cikis yapiliyor..\n");
                    fclose(file);
                    return 1;
                }
                lidar_verisi.ranges[lidar_verisi.ranges_count - 1] = atof(token); //ascıı to float
                token = strtok(NULL, ", \t\n");
            }
        }
        //Normal okuma modu
        else
        {
            // Satırda "ranges = [" var mı diye kontrol eder.Eger yoksa diger satıra devam eder.
            if (strstr(buffer, "ranges = ["))
            {
                rangemod=1;
            }
            else if (strstr(buffer, "angle_min"))
            {
                sscanf(buffer, "angle_min = %lf", &lidar_verisi.angle_min);
            }
            else if (strstr(buffer, "angle_max"))
            {
                sscanf(buffer, "angle_max = %lf", &lidar_verisi.angle_max);
            }
            else if (strstr(buffer, "angle_increment"))
            {
                sscanf(buffer, "angle_increment = %lf", &lidar_verisi.angle_increment);
            }
            else if (strstr(buffer, "range_min"))
            {
                sscanf(buffer, "range_min = %lf", &lidar_verisi.range_min);
            }
            else if (strstr(buffer, "range_max"))
            {
                sscanf(buffer, "range_max = %lf", &lidar_verisi.range_max);
            }
        }
    }

    fclose(file);

    // Okunan verileri ayristir ve kullan
    
    printf("Dosya okuma basarili!\n");
    printf("Angle Min: %f\n", lidar_verisi.angle_min);
    printf("Range Max: %f\n", lidar_verisi.range_max);
    printf("Toplam ranges eleman sayisi: %zu\n", lidar_verisi.ranges_count);

    struct Point* nokta = (struct Point*)malloc(lidar_verisi.ranges_count * sizeof(struct Point)); //noktalar belli olmadıgı icin malloc ile yer ayırdık.

    if (nokta == NULL)
    {
        printf("Noktalar icin bellek ayrilamadi!\n");
        free(lidar_verisi.ranges); // Önceki belleği temizle bellek sızıntısını engellemek icin
        return 1;
    }

    size_t gecerli_nokta_sayisi=0;
    // RANSAC sonuçlarını daha geniş aralıkta yazdırmak icin
    //  Gnuplot kodunun erişebilmesi için burada tanımlıyoruz
#define RANSAC_MAX_LINES 15
    struct RansacLine
    {
        double A, B, C;
        int inliers;
    };
    struct RansacLine dogrular[RANSAC_MAX_LINES];
    int toplam_dogru=0;


    // Gereksiz verileri temizledik
    for (size_t i=0; i < lidar_verisi.ranges_count; ++i)
    {
        float mesafe = lidar_verisi.ranges[i];

        if (isnan(mesafe) || mesafe < lidar_verisi.range_min || mesafe > lidar_verisi.range_max)
        {
            continue; // Geçersiz
        }

        //Gecerli noktaları kordinattan mesafeye cevirelim
        double current_angle_rad = lidar_verisi.angle_min + i * lidar_verisi.angle_increment;

        //Polardan kartezyene
        // x = r * cos(θ)
        // y = r * sin(θ)
        nokta[gecerli_nokta_sayisi].x = mesafe * cos(current_angle_rad);
        nokta[gecerli_nokta_sayisi].y = mesafe * sin(current_angle_rad);

        gecerli_nokta_sayisi++;
    }

    printf("Gecerli nokta sayisi: %zu\n", gecerli_nokta_sayisi);

    //RANSAC başlangıcı (6 doğru bulacak) 

    FILE *dataFile = fopen("noktalar.dat", "w");
    if (dataFile)
    {
        for (size_t i=0; i < gecerli_nokta_sayisi; ++i)
            fprintf(dataFile, "%f %f\n", nokta[i].x, nokta[i].y);
        fclose(dataFile);
    }
    size_t kalan_nokta_sayisi ;
    if (gecerli_nokta_sayisi < 2)
    {
        printf("RANSAC icin yeterli nokta bulunamadi..\n");
    }
    else
    {
        const int ITERATIONS =500;
        const double THRESHOLD =0.05; // metre eşiği
        const int MIN_INLIERS =8;
        const int MAX_LINES =15;

       
        kalan_nokta_sayisi = gecerli_nokta_sayisi;

        printf("\nRANSAC (max %d doğru)\n", MAX_LINES);

        while (kalan_nokta_sayisi > MIN_INLIERS && toplam_dogru < MAX_LINES)
        {
            double best_A =0, best_B =0, best_C =0;
            int max_inliers =0;

            for (int it = 0; it < ITERATIONS; ++it)
            {
                size_t idx1=rand() % kalan_nokta_sayisi;
                size_t idx2;
                double uzaklik=0.0;
                int deneme=0;
                const double MIN_UZAKLIK = 0.03;   // çok yakın olmasın
                const double MAX_UZAKLIK = 0.08;   // çok uzak olmasın
                const int MAX_Deneme=100;          //sonsuzkere yapmasın diye
                do
                {
                    idx2= rand() % kalan_nokta_sayisi;
                    if (idx1 == idx2) continue;

                    double dx= nokta[idx1].x - nokta[idx2].x;
                    double dy= nokta[idx1].y - nokta[idx2].y;
                    uzaklik = sqrt(dx * dx + dy * dy);
                    deneme++;
                }
                while ((uzaklik < MIN_UZAKLIK || uzaklik > MAX_UZAKLIK) && deneme < MAX_Deneme);
                if (deneme >= MAX_Deneme) continue; // uygun nokta çifti bulunamadıysa atla

                struct Point p1 = nokta[idx1];
                struct Point p2 = nokta[idx2];
                if (fabs(p1.x - p2.x) < 1e-9 && fabs(p1.y - p2.y) < 1e-9) continue;

                double A= p2.y - p1.y;
                double B= p1.x - p2.x;
                double C= -A * p1.x - B * p1.y;
                double denom =sqrt(A * A + B * B);
                if (denom < 1e-12) continue;

                int inliers =0;
                for (size_t j = 0; j < kalan_nokta_sayisi; ++j)
                {
                    double d = fabs(A * nokta[j].x + B * nokta[j].y + C) / denom;
                    if (d <= THRESHOLD) inliers++;
                }

                if (inliers > max_inliers)
                {
                    max_inliers =inliers;
                    best_A =A;
                    best_B =B;
                    best_C =C;
                }
            }

            if (max_inliers < MIN_INLIERS) break;

            dogrular[toplam_dogru].A = best_A;
            dogrular[toplam_dogru].B = best_B;
            dogrular[toplam_dogru].C = best_C;
            dogrular[toplam_dogru].inliers = max_inliers;
            toplam_dogru++;

            printf("[%d] Dogru bulundu: %.3fx + %.3fy + %.3f = 0 (%d inlier)\n",toplam_dogru, best_A, best_B, best_C, max_inliers);

            // Inlier noktaları çıkar
            size_t yeni_indeks = 0;
            double denom_best = sqrt(best_A * best_A + best_B * best_B);
            for (size_t j=0; j < kalan_nokta_sayisi; ++j)
            {
                double d = fabs(best_A * nokta[j].x + best_B * nokta[j].y + best_C) / denom_best;
                if (d > THRESHOLD)
                {
                    nokta[yeni_indeks++] = nokta[j];
                }
            }
            kalan_nokta_sayisi = yeni_indeks;
            printf("Kalan nokta sayisi: %zu\n", kalan_nokta_sayisi);
        }

        printf("\nToplam %d dogru bulundu..\n", toplam_dogru);

        if (toplam_dogru == 0)
        {
            printf("Dogru bulunamadi..\n");
        }
        else
        {
            for (int i = 0; i < toplam_dogru; ++i)
            {
                printf("Dogru %d -> %.3fx + %.3fy + %.3f = 0 (%d inlier)\n",
                       i+1, dogrular[i].A, dogrular[i].B, dogrular[i].C, dogrular[i].inliers);
            }

            //Kesisim noktasi hesaplama
            printf("\nKesisim Noktalari\n");
            for (int i=0; i < toplam_dogru; ++i)
            {
                for (int j=i + 1; j < toplam_dogru; ++j)
                {
                    double det = dogrular[i].A * dogrular[j].B - dogrular[j].A * dogrular[i].B;
                    if (fabs(det) < 1e-9)
                        continue; // paralel ya da çakışık doğrular

                    double x= (dogrular[i].B * dogrular[j].C - dogrular[j].B * dogrular[i].C) / det;
                    double y= (dogrular[j].A * dogrular[i].C - dogrular[i].A * dogrular[j].C) / det;

                    printf("Dogru%d ve Dogru%d: (%.3f, %.3f)\n", i, j, x, y);
                    sprintf(dosyaad,"dogru%d_ile_dogru%d_kesisimi.dat",i,j);
                    FILE *kesisimler = fopen(dosyaad,"w");
                    if(!kesisimler)
                    {
                        printf("%s dosyasi acilamadi",dosyaad);
                        continue;
                    }
                    fprintf(kesisimler,"%f %f\n",x,y);
                    fclose(kesisimler);
                    kesnokta[kesnoktasayac].xx=x;
                    kesnokta[kesnoktasayac].yy=y;
                    kesnoktasayac++;
                    sprintf(dosyaad,"dogru%d_ile_dogru%d_kesisim_dogru.dat",i,j);
                    FILE *kesisim_dogru=fopen(dosyaad,"w");
                    if(!kesisim_dogru)
                    {
                        printf("kesiism dogrulari yazdirilamadi");
                        continue;
                    }
                    fprintf(kesisim_dogru,"0 0\n%f %f",x,y);
                    fclose(kesisim_dogru);


                }

            }

            // Robota yakin en yakin dogru ve aci hesaplama
            printf("\nRobotun yakinindaki aci\n");
            if (toplam_dogru >= 2)
            {
                // Her bir doğru ciftini birbiriyle karsilaştirmak için iç içe döngü
                // j = i + 1 optimizasyonu, her çiftin sadece bir kez kontrol edilmesini sağlar.
                int acisira=0;
                for (int i = 0; i < toplam_dogru; i++)
                {
                    for (int j = i + 1; j < toplam_dogru; j++)
                    {

                        //Kesisim analizi
                        double A1= dogrular[i].A, B1 = dogrular[i].B, C1 = dogrular[i].C;
                        double A2= dogrular[j].A, B2 = dogrular[j].B, C2 = dogrular[j].C;

                        double det= A1 * B2 - A2 * B1; // Determinant

                        // Determinant sifira cok yakinsa dogrular paraleldir.
                        if (fabs(det) < 1e-9)
                        {
                            continue; //Atla
                        }

                        // Kesisim noktasini bul
                        double x=(B1 * (-C2) - B2 * (-C1)) / det;
                        double y=(A2 * (-C1) - A1 * (-C2)) / det;

                        // Aciya gore dogru secme

                        double dot_product=A1 * A2 + B1 * B2;
                        double mag1=sqrt(A1 * A1 + B1 * B1);
                        double mag2=sqrt(A2 * A2 + B2 * B2);

                        // cos(theta) = dot / (mag1 * mag2)
                        double aci_rad=acos(dot_product / (mag1 * mag2));
                        aci_deg = aci_rad * 180.0 / M_PI;
                        kesnokta[acisira].acii=aci_deg;
                        acisira++;
                        // Proje İsteği
                        if (aci_deg >= 60.0)
                        {
                            printf("GECERLI KESISIM: Dogru %d ve Dogru %d arasi\n", i + 1, j + 1);
                            printf("  ->Kesisim Noktasi: (X: %.2f, Y: %.2f)\n", -x, -y);
                            printf("  ->Aralarindaki Aci: %.1f derece\n", aci_deg);
                            // Robot (0,0) ile kesisim noktasi arasindaki Öklid mesafesi
                            double distance_to_robot = sqrt(x * x + y * y);
                            printf("  -> Robota Uzaklik: %.2f metre\n\n", distance_to_robot);
                        }
                    }
                }
            }
            else
            {
                printf("Kesisim ve aci analizi icin yeterli dogru bulunamadi.\n");
            }
        }
    }

    printf("\nRANSAC sonlandi\n");

    // RANSAC Son
    //Gnuplot ile görsellestirme


    #define INLIER_THRESHOLD 0.05

    struct Point** dogru_inliers = malloc(toplam_dogru * sizeof(struct Point*));
    int* dogru_inlier_sayilari = calloc(toplam_dogru, sizeof(int));

    if (!dogru_inliers || !dogru_inlier_sayilari)
    {
        printf("Bellek ayrilamadi\n");
        exit(1);
    }

    for (int i = 0; i < toplam_dogru; ++i)
    {
        // Güvenli miktarda bellek ayir
        dogru_inliers[i] =malloc(gecerli_nokta_sayisi * sizeof(struct Point));
        dogru_inlier_sayilari[i]=0;

        double denom = sqrt(dogrular[i].A * dogrular[i].A + dogrular[i].B * dogrular[i].B);

        FILE *tempFile = fopen("noktalar.dat", "r");
        if (!tempFile)
        {
            printf("noktalar.dat dosyasi açilamadi!\n");
            continue;
        }

        double x, y;
        while (fscanf(tempFile, "%lf %lf", &x, &y) == 2)
        {
            double d = fabs(dogrular[i].A * x + dogrular[i].B * y + dogrular[i].C) / denom;
            if (d <= INLIER_THRESHOLD)
            {
                int k = dogru_inlier_sayilari[i]++;
                dogru_inliers[i][k].x =x;
                dogru_inliers[i][k].y =y;
            }
        }
        fclose(tempFile);
    }
    for (int i = 0; i < toplam_dogru; ++i)
    {
        sprintf(dosyaad, "dogru%d.dat", i);
        FILE *lineFile =fopen(dosyaad, "w");
        if (!lineFile)
        {
            printf("%s dosyasi acilamadi!\n", dosyaad);
            continue;
        }

        // Bu dogruya ait inlier noktalarinin sinirlarini bul
        double min_x=1e9, max_x=-1e9;
        double min_y=1e9, max_y=-1e9;
        int inlier_count = 0;

        double denom = sqrt(dogrular[i].A * dogrular[i].A + dogrular[i].B * dogrular[i].B);

        // noktalar.dat dosyasını oku ve bu doğruya ait inlier'ları bul
        FILE *tempFile = fopen("noktalar.dat", "r");
        if (tempFile)
        {
            double x, y;
            while (fscanf(tempFile, "%lf %lf", &x, &y) == 2)
            {
                double d=fabs(dogrular[i].A * x + dogrular[i].B * y + dogrular[i].C) / denom;
                if (d <= 0.05) // THRESHOLD değeri
                {
                    if (x < min_x) min_x=x;
                    if (x > max_x) max_x=x;
                    if (y < min_y) min_y=y;
                    if (y > max_y) max_y=y;
                    inlier_count++;
                }
            }
            fclose(tempFile);
        }

        if (inlier_count < 2)
        {
            fclose(lineFile);
            continue;
        }

        // dogruyu bu sinirlar icinde genislet(biraz genişlet)
        double pay=0.05; // 5cm ek pay
        double x1=min_x - pay;
        double x2=max_x + pay;


        if (fabs(dogrular[i].B) > 1e-9)
        {
            //Normal dogru: y=-(A*x+C)/B
            double y1 = (-dogrular[i].A * x1 - dogrular[i].C) / dogrular[i].B;
            double y2 = (-dogrular[i].A * x2 - dogrular[i].C) / dogrular[i].B;
            fprintf(lineFile, "%f %f\n%f %f\n", x1, y1, x2, y2);
            dogru[i].xx1=x1;
            dogru[i].yy1=y1;
            dogru[i].xx2=x2;
            dogru[i].yy2=y2;
        }
        else if (fabs(dogrular[i].A) > 1e-9)
        {
            //Dikey dogru:x=-C/A
            double x= -dogrular[i].C / dogrular[i].A;
            double y1=min_y - pay;
            double y2=max_y + pay;
            fprintf(lineFile, "%f %f\n%f %f\n", x, y1, x, y2);
            dogru[i].xx1=x;
            dogru[i].yy1=y1;
            dogru[i].xx2=x;
            dogru[i].yy2=y2;
        }

        fclose(lineFile);
    }
    FILE *xxdataFile = fopen("kalannoktalar.dat", "w");
    if (xxdataFile)
    {
        for (size_t i=0; i < kalan_nokta_sayisi; ++i)
            fprintf(xxdataFile, "%f %f\n", nokta[i].x, nokta[i].y);
        fclose(xxdataFile);
    }


    // Bellegi temizle
    for (int i=0; i < toplam_dogru; ++i)
    {
        free(dogru_inliers[i]);
    }
    free(dogru_inliers);
    free(dogru_inlier_sayilari);

    printf("\nGnuplot ile çizim yapiliyor...\n");

    // Gnuplot yolunu belirtme
    const char *gnuplot_path = "\"C:\\\\Program Files\\\\gnuplot\\\\bin\\\\gnuplot.exe\" -persistent";
    char *renkler[] = {"red", "blue", "green", "orange", "purple", "cyan", "magenta", "brown", "black","golden","pink","yellow","dark-olivegreen","web-green","aquamarine","light-goldenrod","beige","honeydew","web-blue","sandybrown","lightcoral","light-salmon","light-cyan","khaki1","lemonchiffon"};
    char *renkler1[]= {"","beige","honeydew","web-blue","sandybrown","lightcoral","light-salmon","light-cyan","khaki1","lemonchiffon"};
    FILE *gnuplotPipe = _popen(gnuplot_path, "w");
   if (gnuplotPipe)
{
    fprintf(gnuplotPipe, "set title 'LIDAR Noktalari ve RANSAC Dogrulari'\n");
    fprintf(gnuplotPipe, "set xlabel 'X (metre)'\n");
    fprintf(gnuplotPipe, "set ylabel 'Y (metre)'\n");
    fprintf(gnuplotPipe, "set grid\n");
    fprintf(gnuplotPipe, "plot 'noktalar.dat' using 1:2 with points pointtype 7 lc rgb 'red' title 'Kulllanılan Lidar Noktalari',\\\n");
    fprintf(gnuplotPipe, "     'kalannoktalar.dat' using 1:2 with points pointtype 7 lc rgb 'gray' title ' Lidar Noktalari',\\\n");
    fprintf(gnuplotPipe, "     'makine.dat' using 1:2 with points pointtype 58 pointsize 3 lc rgb 'black' title ' Makine',\\\n");

    for (int i = 0; i < toplam_dogru; i++)
    {
        fprintf(gnuplotPipe, "     'dogru%d.dat' using 1:2 with lines linewidth 2 lc rgb '%s' title 'RANSAC Dogrusu%d',\\\n", i, renkler[i], i + 1);
    }

    int noktasekli = 1;
    int deneme = 0;
    printf("%d\n", kesnoktasayac);

//En kısa dogruyu saklamak icin
double secilen_x=0.0,secilen_y=0.0,secilen_uzunluk=999999.0,secilen_aci=0.0;
int secilen_i=-1,secilen_j=-1;

//En kisa olani secmek
for (int iii=0; iii < toplam_dogru; iii++)
{
    for (int jjj = iii + 1; jjj < toplam_dogru; jjj++)
    {
        int hepsi_dogru = 0;

        if (kesnokta[deneme].xx > dogru[iii].xx1) hepsi_dogru++;
        if (kesnokta[deneme].xx < dogru[iii].xx2) hepsi_dogru++;
        if (kesnokta[deneme].xx > dogru[jjj].xx1) hepsi_dogru++;
        if (kesnokta[deneme].xx < dogru[jjj].xx2) hepsi_dogru++;

        //Sadece 60 uzerindekileri dikkate al
        if (kesnokta[deneme].acii > 60)
        {
            hepsi_dogru++;

            if (hepsi_dogru >= 5)
            {
                double x=kesnokta[deneme].xx;
                double y=kesnokta[deneme].yy;
                double uzunluk=sqrt(x * x + y * y);
                double aci=kesnokta[deneme].acii;

                // 🔹 En kısa doğrunun seçilmesi
                if (uzunluk < secilen_uzunluk)
                {
                    secilen_x=x;
                    secilen_y=y;
                    secilen_uzunluk=uzunluk;
                    secilen_aci=aci;
                    secilen_i=iii;
                    secilen_j=jjj;
                }
            }
        }

        deneme++;
    }
}

        //en kısa doğruyu çiz
        fprintf(gnuplotPipe,"\n");
        fprintf(gnuplotPipe,"unset label\n");

    if (secilen_i != -1)
    {
    double x_mid=secilen_x/2.0;
    double y_mid=secilen_y/2.0;

    //Terminal ciktisi
    printf("\nMAKİNEYE EN KISA UZAKLIK\n");
    printf("Dogru %d ve Dogru %d arasi\n",secilen_i + 1, secilen_j + 1);
    printf("Kesisim Noktasi: (%.3f , %.3f)\n",secilen_x, secilen_y);
    printf("Aci: %.2f derece\n",secilen_aci);
    printf("Makineye Uzaklik: %.3f metre\n",secilen_uzunluk);

    //txt kaydet
    FILE *out = fopen("makine_uzunluk.txt", "w");
    if (out)
    {
        fprintf(out, "En Kisa Dogru: Dogru%d ve Dogru%d\nKesisim:(%.3f, %.3f)\nAci: %.2f derece\nUzaklik: %.3f m\n",secilen_i + 1, secilen_j + 1, secilen_x, secilen_y, secilen_aci, secilen_uzunluk);
        fclose(out);
    }

    //Makine (0,0) ile kesisim noktasi arasindaki dogruyu çiz
    fprintf(gnuplotPipe,
        "set arrow from 0,0 to %f,%f head filled size screen 0.02,15 lw 3 lc rgb 'red' front\n",secilen_x, secilen_y);

    //Etiketi grafige ekle
    fprintf(gnuplotPipe,
        "set label sprintf('%.2f m - %.1f°', %.2f, %.1f) at %f,%f tc rgb 'blue' font ',11' boxed\n",secilen_uzunluk, secilen_aci,secilen_uzunluk, secilen_aci,x_mid, y_mid);

    fprintf(gnuplotPipe,"replot\n");
    }

    fflush(gnuplotPipe);
    _pclose(gnuplotPipe);
}
else
    {
    printf("Gnuplot acilamadi\n");
    }

    //Gnuplot bolumu sonu 

    free(lidar_verisi.ranges);

    return 0;
}

