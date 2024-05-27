#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include <LiquidCrystal_I2C.h>
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
#include "camera_pins.h"
#include "string.h"

// Provide the RTDB payload printing info and other helper functions.
//#include <addons/RTDBHelper.h>

#define btn_1 15
#define btn_2 2
#define btn_reset 3

#define key 14


#define SDA 12
#define SCL 13
//#define COLUMNS 16
// #define ROWS    2

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;
static mtmn_config_t mtmn_config = { 0 };
static int8_t stream_enabled = 0;
static int8_t detection_enabled = 1;
static int8_t recognition_enabled = 1;
static int8_t is_enrolling = 0;
static int8_t menu = 1;
static int8_t x = 1;
static int8_t reset_mode = 0;
static int8_t wrong_pass = 0;
static int8_t init_state = 0;
static bool reset_enable = false;
static bool home = false;
static bool open = false;
static bool back = false;
int error_code = 0;
int error_recog = 0;
static face_id_list id_list = { 0 };
LiquidCrystal_I2C lcd(0x27, 20, 4);


void detectsMovement(void *arg) {
  if (init_state != 0)
    reset_mode = 1;
}
static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}



static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id) {
  // camera_fb_t *fb = esp_camera_fb_get();
  //       if (fb) {
  //           ESP_LOGE(TAG, "Lỗi khi nhận khung hình");
  //       }
  int x, y, w, h, i;
  uint32_t color = FACE_COLOR_YELLOW;
  if (face_id < 0) {
    color = FACE_COLOR_RED;
  } else if (face_id > 0) {
    color = FACE_COLOR_GREEN;
  }
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {
    // rectangle box
    x = (int)boxes->box[i].box_p[0];
    y = (int)boxes->box[i].box_p[1];
    w = (int)boxes->box[i].box_p[2] - x + 1;
    h = (int)boxes->box[i].box_p[3] - y + 1;
    fb_gfx_drawFastHLine(&fb, x, y, w, color);
    fb_gfx_drawFastHLine(&fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(&fb, x, y, h, color);
    fb_gfx_drawFastVLine(&fb, x + w - 1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
  }
}

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes) {

  dl_matrix3du_t *aligned_face = NULL;
  int matched_id = 0;

  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  if (!aligned_face) {
    Serial.println("Could not allocate face recognition buffer");
    return matched_id;
  }
  if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
    if (is_enrolling == 1) {
      int8_t left_sample_face = enroll_face_id_to_flash(&id_list, aligned_face);

      if (left_sample_face == (ENROLL_CONFIRM_TIMES - 1)) {
        Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
      }
      Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enrolling ID: " + String(id_list.tail));
      lcd.setCursor(0, 1);
      lcd.print("Sample: " + String(ENROLL_CONFIRM_TIMES - left_sample_face));
      lcd.setCursor(0, 3);
      lcd.print("Btn_3:Home/Restart");
      delay(10);

      if (left_sample_face == 0) {
        is_enrolling = 0;
        Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
        is_enrolling = 0;
        stream_enabled = 0;
        menu = 0;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enrolled: " + String(id_list.tail));
        delay(1000);
      }
    } else {

      matched_id = recognize_face(&id_list, aligned_face);
      if (matched_id >= 0) {
        Serial.printf("Match Face ID: %u\n", matched_id);
        error_code = 0;
        error_recog = 0;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Welcome Home");
        delay(1000);
        reset_enable = true;
        stream_enabled = 0;
        wrong_pass = 0;
        // rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", matched_id);
        // Khởi tạo chân GPIO cho đèn flash và cấu hình là đầu ra
        //pinMode(4, OUTPUT)
      } else {
        Serial.println("No Match Found");
        // rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
        matched_id = -1;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("I dont know you");
        lcd.setCursor(0, 1);
        delay(1000);
        lcd.print("You have " + String(5 - error_recog) + " chances");
        lcd.setCursor(0, 3);
        lcd.print("Btn_3:Home/Restart");
        delay(1000);
        error_recog += 1;
      }
    }
  } else {
    Serial.println("Face Not Aligned");
    //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
  }

  dl_matrix3du_free(aligned_face);
  return matched_id;
}
void init_cam() {
  ra_filter_init(&ra_filter, 20);

  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;

  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  read_face_id_from_flash(&id_list);
}
void delete_face() {
  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  read_face_id_from_flash(&id_list);  // Read current face data from on-board flash
  Serial.println("Faces Read");
  while (delete_face_id_in_flash(&id_list) > -1) {
    Serial.println("Deleting Face");
  }
  Serial.println("All Deleted");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("All Deleted");
  delay(1000);
  menu = 0;
}
void run_cam() {
  //error_recog = 0;
  init_state = 1;
  if (reset_mode == 1) { check_reset(); back = false;}
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Start recognition");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Keep distance 20cm");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("Face in cam vision");
  lcd.setCursor(0, 2);
  delay(1000);
  lcd.setCursor(0, 3);
  lcd.print("Btn_3:Home/Restart");
  delay(1000);
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
  bool detected = false;
  int face_id = 0;
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  while (stream_enabled) {

    if (reset_mode == 1) {
      check_reset();
      if(back == true) {back = false;}
      if(reset_enable == false && home==true){
      stream_enabled = 0;
      is_enrolling = 0;
      break;
      }
    }

    if (error_recog > 5) {
      stream_enabled = 0;
      break;
    }
    detected = false;
    face_id = 0;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
      if (!detection_enabled || fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      } else {
        // chuyen matrix 3d (btach size, chieu rong, dai, channel size)
        image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

        if (!image_matrix) {
          Serial.println("dl_matrix3du_alloc failed");
          res = ESP_FAIL;
        } else {
          if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
            Serial.println("fmt2rgb888 failed");
            res = ESP_FAIL;
          } else {
            fr_ready = esp_timer_get_time();
            box_array_t *net_boxes = NULL;
            if (detection_enabled) {
              net_boxes = face_detect(image_matrix, &mtmn_config);
            }
            fr_face = esp_timer_get_time();
            fr_recognize = fr_face;
            if (net_boxes || fb->format != PIXFORMAT_JPEG) {
              if (net_boxes) {
                detected = true;
                if (recognition_enabled) {
                  face_id = run_face_recognition(image_matrix, net_boxes);
                }
                fr_recognize = esp_timer_get_time();
                draw_face_boxes(image_matrix, net_boxes, face_id);
                dl_lib_free(net_boxes->score);
                dl_lib_free(net_boxes->box);
                dl_lib_free(net_boxes->landmark);
                dl_lib_free(net_boxes);
              }
              if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)) {
                Serial.println("fmt2jpg failed");
                res = ESP_FAIL;
                lcd.clear();
              }
              esp_camera_fb_return(fb);
              fb = NULL;
            } else {

              _jpg_buf = fb->buf;
              _jpg_buf_len = fb->len;
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("No face detected");
              lcd.setCursor(0, 1);
              lcd.print("Move other angle");
              lcd.setCursor(0, 3);
              lcd.print("Btn_3:Home/Restart");
              delay(1000);
            }
            fr_encode = esp_timer_get_time();
          }
          dl_matrix3du_free(image_matrix);
        }
      }
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t ready_time = (fr_ready - fr_start) / 1000;
    int64_t face_time = (fr_face - fr_ready) / 1000;
    int64_t recognize_time = (fr_recognize - fr_face) / 1000;
    int64_t encode_time = (fr_encode - fr_recognize) / 1000;
    int64_t process_time = (fr_encode - fr_start) / 1000;

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
    Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
                  (uint32_t)(_jpg_buf_len),
                  (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
                  avg_frame_time, 1000.0 / avg_frame_time,
                  (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
                  (detected) ? "DETECTED " : "", face_id);
  }
  last_frame = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Exit stream");
  delay(1000);
  if (error_recog > 5) { enter_by_code(); }
}


void enter_by_code() {
  if (reset_mode == 1) { check_reset(); back = false;}
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter code");
  delay(2000);
  int8_t passwordArray[] = { 1, 5, 1, 1, 0, 3 };
  int8_t fakeArray[] = { 3, 2, 9, 2, 7, 1 };
  String code = "";
  int8_t position;
  int8_t checkArray[] = { 1, 1, 1, 1, 1, 1 };
  bool hasFalse = false;
  while (error_code != 3) {
    if (reset_mode == 1) {
        check_reset(); back = false;
      }
    if (home) {
      break;
    }
    int8_t checkArray[] = { 1, 1, 1, 1, 1, 1 };
    hasFalse = false;
    for (int i = 0; i < 6; i++) {
      if (home) {
        break;
      }
      code:
      checkArray[i] = 1;
      randomSeed(millis());
      // Random vị trí cho  (0 hoặc 1)
      position = random(2);
      // Random giá trị cho từ 0 đến 9do
      
      lcd.clear();
      delay(500);
      lcd.setCursor(0, 0);
      lcd.print("btn_1: ");
      lcd.setCursor(8, position);
      lcd.print(String(passwordArray[i]));
      lcd.setCursor(0, 1);
      lcd.print("btn_2: ");
      lcd.setCursor(8, 1 - position);
      lcd.print(String(fakeArray[i]));
      lcd.setCursor(0, 2);
      lcd.print("No"+String(i+1));
      lcd.setCursor(0, 3);
      lcd.print("Btn_3:Home/Restart");
      delay(100);

      while (1) {
        if (reset_mode == 1) { 
          check_reset();
          if(home) break;
          if(reset_enable == false && home==false || back == true)
          {
            back = false;
            goto code;
            
          }
          
        }
        if (digitalRead(btn_1) == HIGH) {
          delay(1000);
          if (digitalRead(btn_2) == HIGH) {
            if(i!=0) {i=i-1;}
            else i = 0;
            goto code;
          }
          if (position == 1) { checkArray[i] = 0; }
          break;
        } else if (digitalRead(btn_2) == HIGH) {
          delay(1000);
          if (digitalRead(btn_2) == HIGH) {
            if(i!=0) {i=i-1;}
            else i = 0;
            goto code;
          }
          if (position == 0) { checkArray[i] = 0; }
          break;
        }
      }
    }
    if(home==false){
      for (int i = 0; i < sizeof(checkArray); i++) {
        if (checkArray[i] == 0) {
            hasFalse = true;
            break;
        }
    }
      if (hasFalse == false) {
      wrong_pass = 0;
      reset_enable = true;
      error_code = 0;
      error_recog = 0;
      break;
      } else if (hasFalse == true) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Wrong pass x" + String(error_code - 1));
      lcd.setCursor(0, 0);
      lcd.print("You have " + String(2 - error_code) + " chances");
      delay(1000);
      error_code++;
      reset_enable = false;
      stream_enabled = 0;
      }

    }
    
  }
  // delay 30s cho nhap tiep theo
  if (error_code >= 3) {
    wrong_pass += 1;
    countdownTimer(30 * wrong_pass);
    error_code = 0;
    error_recog = 0;
  }
}

void countdownTimer(int seconds) {
  for (int i = seconds; i >= 0; i--) {
    if (reset_mode == 1) { check_reset(); back = false;}
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Try again");
    lcd.setCursor(0, 1);  // Đặt con trỏ về vị trí đầu tiên của hàng đầu tiên
    lcd.print("Next time: " + String(i));
    digitalWrite(4,1);
    delay(500);  // Dừng một giây trước khi giảm số lượng giây
    digitalWrite(4,0);
    delay(500);  // Dừng một giây trước khi giảm số lượng giây
  }
}

void unlock() {
  //ledcWrite(key, 255); // Điều chỉnh độ sáng của đèn flash

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Unlocking...");
  delay(1000);
  digitalWrite(key, 1);
  delay(1000);
  digitalWrite(key, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Done");
  for (int i = 0; i < 5; i++) {
    delay(100);
  }


  stream_enabled = 0;
  wrong_pass = 0;
  reset_mode = 0;
  //reset so lan sai
}
void lcd_menu_dis() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("btn1:recognition");
  lcd.setCursor(0, 1);
  lcd.print("btn2:setting");
  delay(500);
}
void lcd_init_dis() {
  if (stream_enabled == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("My phone:0383171511");
    lcd.setCursor(0, 1);
    lcd.print("Btn_1:Recognition");
    lcd.setCursor(0, 2);
    lcd.print("Btn_2:Code");
    lcd.setCursor(0, 3);
    lcd.print("Btn_3:Fast Open");
    delay(100);
  }
  reset_enable=false;
  reset_mode=0;
  loop();
}

void check_reset() {
  if (reset_mode == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Btn1:Home");
    lcd.setCursor(0, 1);
    lcd.print("Btn2:Reset");
    lcd.setCursor(0, 2);
    lcd.print("Hold btn1+btn2:Back");
    delay(100);

    while (reset_mode == 1) {
      if (digitalRead(btn_1) == HIGH) {
        delay(1000);
        if (digitalRead(btn_2) == HIGH)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Backing");
          lcd.setCursor(0, 1);
          lcd.print("Release your hand");
          delay(2000);
          back = true;
          reset_mode = 0;
          break;
        }
        reset_mode = 0;
        home = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Home");
        delay(1000);
        lcd_init_dis();
      }
      
      if (digitalRead(btn_2) == HIGH && reset_enable == true) {
        reset_mode = 0;
        delay(1000);
        if (digitalRead(btn_1) == HIGH)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Backing");
          lcd.setCursor(0, 1);
          lcd.print("Release your hand");
          delay(2000);
          back = true;
          reset_mode = 0;
          break;
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Restarting...");
        delay(1000);
        ESP.restart();
      }
      else if (digitalRead(btn_2) == HIGH && reset_enable == false) {
        delay(1000);
        if (digitalRead(btn_2) == HIGH)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Backing");
          lcd.setCursor(0, 1);
          lcd.print("Release your hand");
          delay(2000);
          back = true;
          reset_mode = 0;
          break;
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Can't Restart");
        lcd.setCursor(0, 1);
        lcd.print("Cause dont know you");
        delay(3000);
        reset_mode = 0;
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Wire.begin(SDA, SCL);


  digitalWrite(btn_reset, 0);
  pinMode(btn_1, INPUT);
  pinMode(btn_2, INPUT);
  pinMode(btn_reset, INPUT);

   pinMode(4, OUTPUT);
  pinMode(key, OUTPUT);
  digitalWrite(key, LOW);
  digitalWrite(4, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setting Up");
  delay(1000);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // Đảo ngược ảnh theo chiều dọc
    s->set_brightness(s, 1);   // Tăng độ sáng của ảnh
    s->set_saturation(s, -2);  // Giảm độ bão hòa của ảnh
  }
  // drop down frame size for higher initial frame rate
  //QVGA (320x240 pixel)
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif



  err = gpio_isr_handler_add(GPIO_NUM_3, &detectsMovement, (void *)GPIO_NUM_3);  //
  if (err != ESP_OK) {
    Serial.printf("Không thể thêm trình xử lý ngắt! Lỗi = %d", err);
    return;
  }

  // Thiết lập loại ngắt cho chân GPIO4 (BUTTON_PIN) là ngắt theo cạnh dương
  err = gpio_set_intr_type(GPIO_NUM_3, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("Không thể thiết lập loại ngắt! Lỗi = %d", err);
    return;
  }


  init_cam();
  lcd_init_dis();
}

void loop() {
  init_state = 0;
  if (digitalRead(btn_1) == HIGH || digitalRead(btn_2) == HIGH) {
    delay(200);
    init_state = 1;
    menu = 1;
    x = 1;
    while (1) {
      if (reset_mode == 1) { check_reset(); back = false;}
      
      if (digitalRead(btn_1) == HIGH) {
        delay(200);
        stream_enabled = 1;
        run_cam();
        break;
      } 
      else if (digitalRead(btn_2) == HIGH) {
        if (reset_mode == 1) { check_reset(); back = false;}
        enter_by_code();

        if (wrong_pass > 0) {
          lcd_init_dis();
          return;
        }
        else {
          reset_enable = true;
        }
        break;
      }

      if (wrong_pass > 0) {
        lcd_init_dis();
        return;
      }
    }
    if (home == false) {
      if (wrong_pass == 0) {
      setting1:
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Btn1:open");
        lcd.setCursor(0, 1);
        lcd.print("Btn2:next");
        lcd.setCursor(0, 3);
        lcd.print("Btn3:Home/Reset");
        delay(200);
        while (1) {
          if (reset_mode == 1) {
            check_reset(); 
            if(home== true) break;
            if(back == true) {back = false; goto setting1;}
          }
          if (digitalRead(btn_1) == HIGH) {
            delay(1000);
            
            delay(200);
            unlock();
            break;
          } else if (digitalRead(btn_2) == HIGH) {
            setting2:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Btn1:Enroll");
            lcd.setCursor(0, 1);
            lcd.print("Btn2:Delete");
            lcd.setCursor(0, 2);
            lcd.print("Hold btn1+btn2:Back");
            lcd.setCursor(0, 3);
            lcd.print("Btn3:Home/Reset");
            delay(200);
            while (menu) {
              if (reset_mode == 1) { check_reset();
              if(home== true) break;
              if(back == true) {back = false; goto setting2;}
              }
              if (digitalRead(btn_1) == HIGH) {
                delay(1000);
                if (digitalRead(btn_2) == HIGH) {
                   lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Backing");
                lcd.setCursor(0, 1);
                lcd.print("Release your hand");
                delay(2000);
                goto setting1;
            }
                delay(200);
                is_enrolling = 1;
                stream_enabled = 1;
                run_cam();
                break;
              }

              else if (digitalRead(btn_2) == HIGH) {
                delay(1000);
                if (digitalRead(btn_1) == HIGH) {
                  delay(1000);
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("Backing");
                  lcd.setCursor(0, 1);
                  lcd.print("Release your hand");
                  delay(2000);
                  goto setting1;
            }
                delay(200);
                delete_face();
                break;
              }
              
            }
            break;
          }
        }
      }
    } else {
      home = false;
    }
    lcd_init_dis();
  }
  else if (digitalRead(btn_reset) == HIGH)
  {
        delay(500);
        stream_enabled = 1;
        reset_mode=0;
        run_cam();
        if(reset_enable==true){unlock();}
        home=false;
        lcd_init_dis();
        return;
      
  }
}
