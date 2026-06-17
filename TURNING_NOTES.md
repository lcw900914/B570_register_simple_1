# 循跡轉彎 — 目前進度與待解問題

## 未解問題
**連續兩個 90° 彎(尤其第二個)過不去**:會「前進倒退鬼打牆」或「衝出蹌道」,
轉彎非常緩慢甚至轉不過去。各種倒退/煞停/冷卻參數都調過,還是不行。

## 目前的過彎機制(都在 `HARDWARE/IR/ir_tracker.h` + `BALANCE/CONTROL/control.c`)
1. **任何偏離中央就原地轉**:011/001/110/100(只要不是置中 010)→ 觸發原地慢轉 pivot
   (`HARD_TURN_TANK_PIVOT`, `PIVOT_OUTER/INNER`)。
2. **進彎先煞停再轉** (`CORNER_BRAKE_STOP/SPEED`):要轉但車還在前進時,先反向煞停,
   幾乎停了才開始轉,想避免衝過頂點。
3. **轉彎後直走一小段** (`POST_PIVOT_STRAIGHT_CYCLES/SPEED`):轉完不轉向、不 pivot,
   直線前進約 0.3s,讓它順上新線。(原本是「倒退」,後來改成「直走」。)
4. **一個彎只觸發一次** (`backup_lock` + `POST_PIVOT_MIN_TURN` + `POST_PIVOT_REARM`):
   轉夠久才算真彎;觸發後上鎖,要連續乾淨置中 30 cycle 才解鎖,避免重複觸發。

## 試過但仍無效
- 轉彎後倒退(長度 50~180 cycle、速度 3~6 都試過)→ 倒不夠 / 鬼打牆。
- 冷卻(連續置中才解鎖)→ 還是前後。
- 「轉夠久才倒退」門檻 → 急彎每次重轉都夠久,還是每次都觸發。
- 進彎先煞停 → 還是衝出去。
- 轉彎後改直走(現狀)→ 還是不行。

## 目前參數現值
- `PIVOT_OUTER/INNER = 850`(純原地自轉)
- `CORNER_PIVOT_CHUNK = 3`
- `CORNER_BRAKE_STOP = 2`, `CORNER_BRAKE_SPEED = 4`
- `POST_PIVOT_STRAIGHT_CYCLES = 30`, `POST_PIVOT_STRAIGHT_SPEED = 3`
- `POST_PIVOT_MIN_TURN = 15`, `POST_PIVOT_REARM = 30`
- `LINE_BASE_SPEED = 2`
- `VELOCITY_KP_BAL = -100`, `VELOCITY_KI = -0.2`(平衡用速度環,先前為了壓暴衝調小)

## 下次可試的方向
1. **加診斷**:把 `in_pivot / turn_run / post_pivot_straight / corner_braking / Encoder_Left+Right`
   即時顯示到 OLED 或 UART,過彎時錄下來看「到底卡在哪個階段」——目前都在盲調。
2. **懷疑根因 = 慣性煞不住**:速度環太弱(KP_BAL -100)→ 進彎煞不停 → 衝過頂點。
   可試把 `VELOCITY_KP_BAL` 加回(-100 → -150/-200)看煞車是否變強(注意可能回到暴衝)。
3. **整體降速**:`LINE_BASE_SPEED 2 → 1`,減少進彎慣性。
4. **感測器前瞻只有 ~2cm**:90° 彎幾乎一進就 000(丟線),先天難。可考慮加裝感測器/前瞻,
   或專門強化「000 丟線恢復」那條(reverse 把彎角拉回 sensor 下)。
5. **`CORNER_BRAKE_STOP` 單位是編碼器 L+R/10ms**,實際數值未量測;若「一進彎永遠在煞不開始轉」
   代表門檻設太低,要照實測調。

## 關鍵檔案
- 轉彎/循跡參數:`HARDWARE/IR/ir_tracker.h`
- 過彎狀態機 + 馬達混合:`BALANCE/CONTROL/control.c`(EXTI ISR 內)
- 速度環/平衡增益:`BALANCE/CONTROL/control.h`
