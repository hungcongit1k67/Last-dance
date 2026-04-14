import os
from dotenv import load_dotenv
from groq import Groq

# 1. Load API Key từ file .env
load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")

# 2. Đọc và NÉN bản đồ (Xóa dấu cách để giảm Token)
map_path = r'E:\last_dance\LastDance\map\grid30.txt'
# compact_map = ""

# with open(map_path, 'r') as f:
#     for line in f:
#         # Loại bỏ dấu cách và xuống dòng để nén dữ liệu
#         compact_map += line.replace(" ", "").strip() + "\n"

with open(map_path, 'r') as f:
    map_content = f.read()

sample_map = """
2 0 0 0 0
0 1 1 1 0
0 1 1 1 2
0 1 1 1 0
0 0 0 0 0
"""

# 3. Khởi tạo Client Groq
client = Groq(api_key=GROQ_API_KEY)

# 4. Gửi yêu cầu (Sử dụng model 3.3-70b mới nhất)
try:
    print(f"[*] Đang gửi yêu cầu... (Kích thước bản đồ nén: {len(map_content)} ký tự)")
    
    completion = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[
            {
                "role": "system", 
                "content": f"""Bạn là một robot vận chuyển. Đây là bản đồ kho hàng dạng grid_map (0:trống, 1:vật cản, 2:mục tiêu). Ví dụ bản đồ:
{sample_map}
Trong đó tọa độ các điểm 2 của bản đồ trên là (0,0) và (4,2).
Nhiệm vụ: Xác định tọa độ các điểm waypont "2" Tìm lộ trình ngắn nhất xuất phát từ một điểm "2" bất kỳ và đi qua toàn bộ các điểm '2' và tránh '1'."""
            },
            {
                "role": "user", 
                "content": f"Hãy phân tích tọa độ các điểm '2' và cho tôi danh sách tọa độ (x,y) của đường đi ngắn nhất tối ưu của map {map_content}."
            }
        ],
        temperature=0.1,
        # Giảm max_tokens phản hồi một chút để tránh tổng request + response vượt limit
        max_tokens=8000
    )

    print("\n=== KẾT QUẢ TỪ LLM ===")
    print(completion.choices[0].message.content)

except Exception as e:
    if "413" in str(e) or "rate_limit" in str(e):
        print("\n[LỖI] Bản đồ vẫn quá lớn so với giới hạn miễn phí của Groq.")
        print("Giải pháp: Bạn nên sử dụng phương pháp 'Strategic FMF' (chỉ gửi tọa độ điểm 2 cho LLM) thay vì gửi toàn bộ bản đồ.")
    else:
        print(f"\n[LỖI] {e}")