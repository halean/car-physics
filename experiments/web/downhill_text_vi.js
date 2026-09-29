// Lăn xuống dốc: chữ tiếng Việt và cách viết số (dấu phẩy thập phân, dấu chấm phân nhóm).
// Thuật ngữ theo Chương trình giáo dục phổ thông 2018 môn Vật lí (lớp 10) và sách giáo khoa:
// trọng lực P = mg, phản lực N, lực ma sát, định luật 2 Newton, góc nghiêng α, động năng, động lượng.
const num = (v, d) => v.toFixed(d).replace('.', ',');                 // 0,78
const grouped = (v) => Math.round(v).toLocaleString('vi-VN');          // 5.886
const TEXT = {
  uphill: (n) => `${n}, hướng lên dốc`,
  downhill: (n) => `${n}, hướng xuống dốc`,
  agree: (n) => `Hai vế bằng nhau (sai khác dưới ${n} N).`,
  settling: 'Đang ổn định… (tốc độ xe đang thay đổi nhanh)',
  slow: 'Chạy chậm', normal: 'Tốc độ thường',
  loading: (p) => `Đang tải bộ mô phỏng vật lí MuJoCo… ${p}%`,
  loadFail: (name, code) => `Không tải được tệp ${name} (HTTP ${code}).`,
  starting: 'Đang khởi động MuJoCo…',
  failed: (m) => `Không khởi động được mô phỏng: ${m}`,
  noWasm: 'Trình duyệt này không hỗ trợ WebAssembly.',
  speedLimit: (kmh) => `Đã dừng đồng hồ ở ${kmh} km/h. Một chiếc xe năm 1891 không thể chạy nhanh như vậy. Mô hình này `
    + 'bỏ qua lực cản của không khí và lực ma sát lăn, nên trên dốc xe sẽ nhanh dần mãi. Nhấn Chạy lại (R).',
  cowAhead: (m) => `Có bò trên đường, cách ${m} m phía trước`,
  closeCall: 'Suýt nữa! Con bò phải chạy vội ra khỏi đường. Hãy phanh sớm hơn hoặc mạnh hơn.',
  stoppedBefore: (m) => `Xe dừng cách con bò ${m} m.`,
  getReady: 'Chuẩn bị! Phía trước sẽ có một con bò bất ngờ bước ra đường. Nhấn B (hoặc nút Phanh) ngay khi nhìn thấy nó.',
  cow: 'Bò!',
  bang: 'RẦM!',
  reaction: 'Thời gian phản ứng của em',
  thinking: 'Quãng đường phản ứng (xe vẫn chạy với tốc độ cũ trong lúc em phản ứng)',
  braking: 'Quãng đường phanh',
  prediction: 'Dự đoán theo công thức v² = 2as',
  predictionValue: (s, v, a) => `${s} m (v = ${v} m/s, a = ${a} m/s²)`,
  stopping: 'Tổng quãng đường dừng xe',
  whyLonger: 'Quãng đường phanh thực tế dài hơn v² / 2a: má phanh cần một phần tư giây để ép chặt, và các bánh xe đang quay cũng mang động năng mà phanh phải tiêu hao.',
  tooClose: 'Quá gần: con bò phải chạy vội ra khỏi đường. Hãy phanh sớm hơn, hoặc tăng hệ số ma sát μ của phanh.',
  youStopped: (m) => `Em đã dừng xe cách con bò ${m} m.`,
  cannotStop: 'Phanh không giữ được xe trên dốc này: 2μQ nhỏ hơn P sin α = mg sin α. Hãy giảm góc nghiêng hoặc tăng lực phanh.',
  impactSpeed: 'Tốc độ lúc va chạm',
  impactValue: (v, kmh) => `${v} m/s (${kmh} km/h)`,
  kinetic: 'Động năng Wđ = ½mv²',
  kineticValue: (kj, h) => `${kj} kJ, bằng thế năng khi thả rơi xe từ độ cao ${h} m`,
  momentum: (kg) => `Va chạm mềm với con bò ${kg} kg (bảo toàn động lượng)`,
  momentumValue: (v1, v2) => `tốc độ xe giảm từ ${v1} xuống ${v2} m/s`,
  lateBrake: 'Em đã phanh nhưng quá muộn. Thử lại và phanh ngay khi con bò xuất hiện.',
  noBrake: 'Em chưa phanh. Thử lại và nhấn B ngay khi con bò xuất hiện.',
  cartoon: 'Con bò không nằm trong mô hình vật lí: trang web coi đây là va chạm mềm trong hệ kín, '
    + 'm₁v₁ = (m₁ + m₂)V, rồi cho con bò lộn nhào như phim hoạt hình. Con bò không sao, chỉ hơi bực mình.',
};
