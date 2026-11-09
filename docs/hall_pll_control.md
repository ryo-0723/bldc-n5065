# Hall / PLL・Fast速度制御

現行配布版: pll_fast_release_v1。pll_fast_gain_v1の制御仕様を維持し、不要なHallエッジ速度推定・診断状態・追加Terminal診断を削除した版。版の識別は配布BINのファイル名とSHA256で行う。

## 構成

- motor/foc_math.c: 不均等Hall区間の角度補正、PLL/Fast速度選択、速度PID・ゲイン補間。
- motor/foc_speed_source.c/.h: Hall tableの妥当性確認、速度選択のヒステリシス・混合状態、速度/ゲイン切替定数。
- motor/mcpwm_foc.c: ADC処理への接続、通常RPM API、設定更新時の初期化。
- tests/hall_pll/: 現行制御の試験と成果物検証。

制御に使わなかったHallエッジ由来の速度、取得待ちblend、区間履歴、異常理由カウンターを削除。角度制御に必要なHall区間計算は維持する。旧実験用の再生・遅延比較・簡易プラント試験6ファイルと旧仕様の説明書を削除した。整理前のソース一式は配布物のbefore_source.tar.gzへ退避している。

## 有効条件と設定

Hall + FOC Speed Source CORRECTED + Speed PID Speed Source Fastで自動切替する。PLLを選ぶと全域PLL。非Hall、Observer、Faster、位相オーバーライド時の既存経路を維持する。

S=Sensorless ERPM(foc_sl_erpm)。既存PLLの絶対速度で判定し、加速S+700以上でFast、減速S+200以下でPLLへ戻る。50msで速度推定を混合する。S1800なら2500/2000。正逆共通。両推定器の数式と継続更新は従来どおり。

PLL側のKp/Kdと全域のKiはTool設定値。Fast移行開始でKdを0とし、PLL復帰の混合中も0を維持する。KpはS+700から実測PLL絶対速度10000ERPMまで、Tool値から0.00265へ線形補間する。速度blendとゲインの変化率制限(全幅50ms)も適用する。10000以上は0.00265。S+700が10000以上の特殊な設定では、その開始点から最低1ERPMの幅を用いる。

Ki有効時はKp変更とD無効化による出力段差を積分項で補償する。積分項の上限では完全補償を保証しない。Ki0の場合は補償による積分を作らない。後段電流制限を追跡する追加アンチワインドアップや電流指令の新しい変化率制限は導入していない。

Kd0で運用する場合はToolで0を設定する。Kiの値はFW内に固定しない。設定XML/EEPROM形式・電流制限・Hall tableは変更していない。

## Hall角度補正

固定60度ではなく、隣接Hall table代表角の中点を近似境界とし、境界間角度を対応する実測時間で割る。現在区間幅を使って補間継続を判定し、方向別の出口境界まで進めて保持する。逆方向への引き戻しを行わず、従来の角度レート制限とHall/observer角度混合を維持する。

Hall tableは境界角ではなく各状態の代表角。真の区間幅はテーブルだけでは一意に復元できず、中点近似と検出誤差は残る。今回の整理で、この角度処理・PLL・PIDの制御定数を変更していない。

## 診断機能の削除

追加したhall_speed / hall_speed traceコマンド、登録、コールバックを削除した。
terminal.cのHall境界詳細表示・追加記録・測定数チェックも削除し、このファイルは公式7.00基点e877eaeaと完全に一致する。公式FWに元からあるTerminal機能は維持している。

追加制御コードは周辺のタブ字下げ・ifの波括弧・演算子とカンマの空白・継続行の書式へ合わせた。既存のencoder処理や加速ランプ処理に残っていた書式だけの差分は元へ戻した。

## 検証

    make -C tests/hall_pll run
    python3 tests/hall_pll/check_angles.py
    python3 tests/hall_pll/check_cleanup.py BEFORE_SOURCE_ARCHIVE
    make -j8 fw_60 BUILD_DIR=/home/user/bldc-n5065/build/pll-fast-release-v1 DEPPATH=/home/user/bldc-n5065/build/pll-fast-release-v1/60/.dep
    python3 tests/hall_pll/check_firmware.py build/pll-fast-release-v1/60

check_cleanup.pyは変更前のbefore_source.tar.gzが必要。旧Hallエッジ版と整理後の速度切替版の両方を読み込める。12条件、36000 PIDサンプルで角度・選択速度・電流指令・積分・Dフィルタ・ゲイン補間率・実効ゲインが整理前とビット単位で一致することを検証した。加減速、反転、Hall無効値、テーブル変更・不正値、推定ソース変更、Ki0、Kd0、ブレーキ禁止、ゼロ初期化、位相オーバーライド、制御停止を含む固定入力試験。実機の割り込みタイミングや閉ループ安定性の同等性を証明するものではない。

通常の回帰試験、ASan/UBSan、既存Hall角度軌道比較、VESC60ビルド、ソース鮮度・構造体レイアウト・BIN/HEX/ELF整合性を確認。

motor_all_state_tは1176から1072bytesへ縮小(1状態あたり104bytes削減)。診断削除版BINの実機書き込み・起動は未確認。実機で試した旧版BINはartifacts/pll_fast_gain_v1/に保持。
