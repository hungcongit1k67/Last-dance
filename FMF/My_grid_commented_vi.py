# [Giải thích dòng gốc 1] Import `print_form` từ module `cgi`. Trong file này gần như không thấy dùng đến, nên đây có vẻ là import thừa từ phiên bản cũ.
from cgi import print_form
# [Giải thích dòng gốc 2] Import thư viện `random` của Python để sinh số ngẫu nhiên kiểu chuẩn.
import random
# [Giải thích dòng gốc 3] Dòng import đã bị comment lại; trước đây có thể tác giả từng định dùng `tile` của NumPy nhưng hiện không dùng.
#from numpy.lib.shape_base import tile
# [Giải thích dòng gốc 4] Import `pygame` để tạo cửa sổ và vẽ/nhận sự kiện chuột khi thao tác với lưới.
import pygame
# [Giải thích dòng gốc 5] Import NumPy và đặt bí danh là `np` để xử lý mảng số, vector và ma trận.
import numpy as np
# [Giải thích dòng gốc 6] Import module `math` để dùng căn bậc hai và các phép toán toán học cơ bản.
import math
# [Giải thích dòng gốc 7] Import `matplotlib.pyplot` với tên `plt` để vẽ map, wavefront và quỹ đạo.
import matplotlib.pyplot as plt
# [Giải thích dòng gốc 8] Import module `queue` để dùng `PriorityQueue` trong các bước lan truyền và Dijkstra.
import queue 
# [Giải thích dòng gốc 9] Import tên `color` từ `pygame`; thực tế trong file hầu như không dùng nên cũng có thể là import thừa.
from pygame import color

# [Giải thích dòng gốc 11] Định nghĩa lớp `GridMap`, nơi gom toàn bộ dữ liệu map, các thuật toán lan truyền FM/FMF, dựng graph và hàm vẽ.
class GridMap():

    # [Giải thích dòng gốc 13] Khai báo biến mức lớp `mapSize`; sẽ lưu kích thước map hình vuông.
    mapSize = None
    # [Giải thích dòng gốc 14] Khai báo biến mức lớp `gridMap`; sẽ chứa ma trận ô của bản đồ.
    gridMap = None
    # [Giải thích dòng gốc 15] Khai báo biến mức lớp `npos`; số lượng goal/checkpoint trên map.
    npos = None
    # [Giải thích dòng gốc 16] Khai báo biến mức lớp `deslist`; danh sách tọa độ các goal.
    deslist = None
    # [Giải thích dòng gốc 17] Khai báo biến mức lớp `dista`; trường khoảng cách/arrival time khi lan sóng.
    dista = None
    # [Giải thích dòng gốc 18] Khai báo biến mức lớp `trace`; lưu dấu vết cha để lần ngược đường đi.
    trace = None
    # [Giải thích dòng gốc 19] Khai báo biến mức lớp `owner`; mỗi ô thuộc về goal/firework nào.
    owner = None
    # [Giải thích dòng gốc 20] Khai báo biến mức lớp `hold`; nhóm các ô thuộc từng component/goal.
    hold = None
    # [Giải thích dòng gốc 21] Khai báo biến mức lớp `adj`; ma trận trọng số giữa các goal trong graph.
    adj = None
    # [Giải thích dòng gốc 22] Khai báo biến mức lớp `inters`; lưu cặp điểm giao nhau tạo cạnh giữa hai goal.
    inters = None
    # [Giải thích dòng gốc 23] Thiết lập kích thước marker mặc định khi vẽ bằng matplotlib.
    mksz = 10
    # [Giải thích dòng gốc 24] Gán nhãn mặc định cho thuật toán hiện hành là `Fast Marching` để hiển thị trên hình.
    DFType = "Fast Marching"
    # [Giải thích dòng gốc 25] Danh sách màu dùng để tô các vùng/miền do từng goal chiếm giữ.
    colorHold = ['blue','green','gold','tan','maroon','orange','cyan','violet','salmon','lime','darkslateblue']
    # [Giải thích dòng gốc 26] Bảng màu cũ cho sóng lan bị comment lại, có lẽ để thử nghiệm hiển thị.
    #colorWave = ['lightseagreen','aquamarine','aqua','mediumspringgreen','springgreen','blueviolet','violet','slateblue','darkslateblue','plum']
    # [Giải thích dòng gốc 27] Bảng màu mới nhạt hơn để hiển thị lớp sóng hoặc gradient cho dễ nhìn.
    colorWave = ['linen','peachpuff','papayawhip','oldlace','floralwhite','seashell','mintcream','azure','ghostwhite','lightcyan','aliceblue',]
    

    # [Giải thích dòng gốc 30] Mảng độ dịch theo trục x/hàng cho 8 hướng lân cận: 4 thẳng + 4 chéo.
    tx = [-1,0,1,0,-1,1,1,-1]
    # [Giải thích dòng gốc 31] Mảng độ dịch theo trục y/cột tương ứng với `tx` cho 8 hướng lân cận.
    ty = [0,1,0,-1,1,1,-1,-1]

    # [Giải thích dòng gốc 33] Comment gốc: ô có giá trị 2 được hiểu là checkpoint/goal.
    # gird[i][j] = 2 ~ checkpoint
    # [Giải thích dòng gốc 34] Comment gốc: ô có giá trị 1 được hiểu là vật cản.
    # grid[i][j] = 1 ~ block

    # [Giải thích dòng gốc 36] Hàm khởi tạo đối tượng `GridMap`; nhận kích thước map và thông số hiển thị ô lưới.
    def __init__(self, mapSize, square_width = 20, square_height = 20, margin = 1):
        # [Giải thích dòng gốc 37] Lưu kích thước bản đồ vào biến đối tượng.
        self.mapSize = mapSize
        # [Giải thích dòng gốc 38] Lưu chiều rộng mỗi ô khi vẽ bằng `pygame`.
        self.square_width = square_width
        # [Giải thích dòng gốc 39] Lưu chiều cao mỗi ô khi vẽ bằng `pygame`.
        self.square_height = square_height
        # [Giải thích dòng gốc 40] Lưu độ dày khoảng cách giữa các ô khi vẽ.
        self.margin = margin
        # [Giải thích dòng gốc 41] Tính chiều ngang cửa sổ hiển thị dựa trên số ô, kích thước ô và lề.
        self.window_size = [self.mapSize*square_width+(self.mapSize+1)*self.margin,
                            # [Giải thích dòng gốc 42] Tính chiều cao cửa sổ hiển thị theo công thức tương tự cho trục dọc.
                            self.mapSize*square_height+(self.mapSize+1)*self.margin]
        # [Giải thích dòng gốc 43] Khởi tạo map rỗng; sau này sẽ được điền bằng file hoặc thao tác chuột.
        self.gridMap = []
        # [Giải thích dòng gốc 44] Tạo ma trận đánh dấu phục vụ cache cho các phép tìm đường kiểu BFS ở phần cuối file.
        self.checked = np.zeros((mapSize,mapSize))



    # [Giải thích dòng gốc 48] Hàm tạo bản đồ thủ công bằng cách click chuột vào cửa sổ `pygame`.
    def create_grid_map(self,npos):
        # [Giải thích dòng gốc 49] Lưu số lượng goal thực tế có trong file.
        self.npos = npos
        # [Giải thích dòng gốc 50] Khởi tạo mảng tọa độ goal, mỗi goal có 2 thành phần là hàng và cột.
        self.deslist = np.zeros((npos,2))
        # [Giải thích dòng gốc 51] Sao chép chiều rộng ô sang biến cục bộ để code vẽ gọn hơn.
        WIDTH = self.square_width
        # [Giải thích dòng gốc 52] Sao chép chiều cao ô sang biến cục bộ.
        HEIGHT = self.square_height
        # [Giải thích dòng gốc 53] Sao chép khoảng cách giữa các ô sang biến cục bộ.
        MARGIN = self.margin
        # [Giải thích dòng gốc 54] Khởi tạo danh sách 2 chiều để chứa map.
        grid = []
        # [Giải thích dòng gốc 55] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for row in range(self.mapSize):
            # [Giải thích dòng gốc 56] Tạo một hàng mới trong ma trận map.
            grid.append([])
            # [Giải thích dòng gốc 57] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for column in range(self.mapSize):
                # [Giải thích dòng gốc 58] Mặc định điền ô trống bằng giá trị 0.
                grid[row].append(0)

        # [Giải thích dòng gốc 60] Khởi tạo toàn bộ module cần thiết của `pygame`.
        pygame.init()
        # [Giải thích dòng gốc 61] Lấy kích thước cửa sổ đã tính sẵn.
        window_size = self.window_size
        # [Giải thích dòng gốc 62] Tạo cửa sổ hiển thị `pygame` với kích thước tương ứng.
        scr = pygame.display.set_mode(window_size)
        # [Giải thích dòng gốc 63] Đặt tiêu đề cửa sổ là `Grid Map`.
        pygame.display.set_caption("Grid Map")
        # [Giải thích dòng gốc 64] Biến cờ điều khiển vòng lặp giao diện; `False` nghĩa là cửa sổ còn chạy.
        done = False
        # [Giải thích dòng gốc 65] Tạo đồng hồ để khống chế tốc độ cập nhật khung hình.
        clock = pygame.time.Clock()

        # [Giải thích dòng gốc 67] Biến đếm số goal đã nhập.
        i = 0

        # [Giải thích dòng gốc 69] Vòng lặp chính của cửa sổ nhập map bằng chuột.
        while not done:

            # [Giải thích dòng gốc 71] Duyệt tất cả sự kiện phát sinh từ cửa sổ `pygame`.
            for event in pygame.event.get():
                # [Giải thích dòng gốc 72] Kiểm tra xem người dùng có đóng cửa sổ hay không.
                if event.type == pygame.QUIT:
                    # [Giải thích dòng gốc 73] Nếu có yêu cầu đóng cửa sổ thì thoát vòng lặp chính.
                    done = True
                # [Giải thích dòng gốc 74] Xử lý sự kiện bấm chuột vào map.
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # [Giải thích dòng gốc 75] Lấy tọa độ pixel hiện tại của con trỏ chuột.
                    pos = pygame.mouse.get_pos()
                    # [Giải thích dòng gốc 76] Đổi tọa độ pixel x sang chỉ số cột của ô lưới.
                    column = pos[0] // (WIDTH + MARGIN)
                    # [Giải thích dòng gốc 77] Đổi tọa độ pixel y sang chỉ số hàng của ô lưới.
                    row = pos[1] // (HEIGHT + MARGIN)
                    # [Giải thích dòng gốc 78] Nếu chưa nhập đủ goal thì click hiện tại sẽ được xem là một goal.
                    if (i < npos):
                        # [Giải thích dòng gốc 79] Đánh dấu ô đang click là goal/checkpoint bằng giá trị 2.
                        grid[row][column] = 2
                        # [Giải thích dòng gốc 80] Lưu tọa độ goal thứ `i` vào danh sách đích.
                        self.deslist[i] = np.array([row,column])
                        # [Giải thích dòng gốc 81] Tăng biến đếm goal đã nhập.
                        i = i+1
                        # [Giải thích dòng gốc 82] In ra console để kiểm tra nhanh vị trí click theo pixel và theo lưới.
                        print("Click ", pos, "Grid coordinates: ", row, column)
                    # [Giải thích dòng gốc 83] Nếu đã đủ goal thì các click tiếp theo sẽ được hiểu là vật cản.
                    else:
                        # [Giải thích dòng gốc 84] Đánh dấu ô đó là obstacle/block.
                        grid[row][column] = 1
                        # [Giải thích dòng gốc 85] In ra console để kiểm tra nhanh vị trí click theo pixel và theo lưới.
                        print("Click ", pos, "Grid coordinates: ", row, column)
            # [Giải thích dòng gốc 86] Xóa khung hình cũ bằng nền đen trước khi vẽ lại toàn bộ map.
            scr.fill((0,0,0))
            # [Giải thích dòng gốc 87] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for row in range(self.mapSize):
                # [Giải thích dòng gốc 88] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
                for column in range(self.mapSize):
                    # [Giải thích dòng gốc 89] Mặc định màu của ô là trắng, tương ứng ô trống.
                    color = (255,255,255)
                    # [Giải thích dòng gốc 90] Nếu ô hiện tại là vật cản thì đổi màu.
                    if grid[row][column] == 1:
                        # [Giải thích dòng gốc 91] Đặt vật cản màu đen.
                        color = (0,0,0)
                    # [Giải thích dòng gốc 92] Bắt đầu vẽ hình chữ nhật biểu diễn một ô lên bề mặt `scr`.
                    pygame.draw.rect(scr,
                                     # [Giải thích dòng gốc 93] Truyền màu đã chọn ở trên cho ô đang vẽ.
                                     color,
                                     # [Giải thích dòng gốc 94] Tính toạ độ x góc trái trên của ô theo cột.
                                     [(MARGIN + WIDTH) * column + MARGIN,
                                      # [Giải thích dòng gốc 95] Tính toạ độ y góc trái trên của ô theo hàng.
                                      (MARGIN + HEIGHT) * row + MARGIN,
                                      # [Giải thích dòng gốc 96] Chiều rộng của ô cần vẽ.
                                      WIDTH,
                                      # [Giải thích dòng gốc 97] Chiều cao của ô cần vẽ.
                                      HEIGHT])

                    # [Giải thích dòng gốc 99] Mặc định màu của ô là trắng, tương ứng ô trống.
                    color = (255,255,255)
                    # [Giải thích dòng gốc 100] Nếu ô là goal thì sẽ vẽ chồng một hình chữ nhật màu riêng.
                    if (grid[row][column] == 2):
                        # [Giải thích dòng gốc 101] Màu xanh lá để biểu diễn goal/checkpoint.
                        color = (0,255,0)
                        # [Giải thích dòng gốc 102] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
                        pygame.draw.rect(scr,
                                         # [Giải thích dòng gốc 103] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
                                         color,
                                         # [Giải thích dòng gốc 104] Tính lại toạ độ x của ô goal.
                                         [(MARGIN + WIDTH) * column + MARGIN,
                                          # [Giải thích dòng gốc 105] Tính lại toạ độ y của ô goal.
                                          (MARGIN + HEIGHT) * row + MARGIN,
                                          # [Giải thích dòng gốc 106] Chiều rộng của ô goal.
                                          WIDTH,
                                          # [Giải thích dòng gốc 107] Chiều cao của ô goal.
                                          HEIGHT])

            # [Giải thích dòng gốc 109] Giới hạn tốc độ vòng lặp giao diện ở khoảng 50 FPS.
            clock.tick(50)
            # [Giải thích dòng gốc 110] Đẩy toàn bộ nội dung đã vẽ lên màn hình.
            pygame.display.flip()

        # [Giải thích dòng gốc 112] Thoát các module `pygame` khi người dùng đóng cửa sổ.
        pygame.quit()
        # [Giải thích dòng gốc 113] Lưu ma trận map vừa nhập tay vào thuộc tính đối tượng.
        self.gridMap = grid
        # [Giải thích dòng gốc 114] Trả lại map cho nơi gọi hàm nếu cần dùng ngay.
        return grid

        
    # [Giải thích dòng gốc 117] Hàm chỉ để hiển thị map hiện có, không cho chỉnh sửa cấu trúc map.
    def showGridMap(self):

        # [Giải thích dòng gốc 119] Sao chép chiều rộng ô sang biến cục bộ để code vẽ gọn hơn.
        WIDTH = self.square_width
        # [Giải thích dòng gốc 120] Sao chép chiều cao ô sang biến cục bộ.
        HEIGHT = self.square_height
        # [Giải thích dòng gốc 121] Sao chép khoảng cách giữa các ô sang biến cục bộ.
        MARGIN = self.margin
        # [Giải thích dòng gốc 122] Lấy map hiện tại của đối tượng ra biến cục bộ để dùng khi vẽ.
        grid = self.gridMap

        # [Giải thích dòng gốc 124] Khởi tạo toàn bộ module cần thiết của `pygame`.
        pygame.init()
        # [Giải thích dòng gốc 125] Lấy kích thước cửa sổ đã tính sẵn.
        window_size = self.window_size
        # [Giải thích dòng gốc 126] Tạo cửa sổ hiển thị `pygame` với kích thước tương ứng.
        scr = pygame.display.set_mode(window_size)
        # [Giải thích dòng gốc 127] Đặt tiêu đề cửa sổ hiển thị là `Grid`.
        pygame.display.set_caption("Grid")
        # [Giải thích dòng gốc 128] Biến cờ điều khiển vòng lặp giao diện; `False` nghĩa là cửa sổ còn chạy.
        done = False
        # [Giải thích dòng gốc 129] Tạo đồng hồ để khống chế tốc độ cập nhật khung hình.
        clock = pygame.time.Clock()
        

        # [Giải thích dòng gốc 132] Vòng lặp chính của cửa sổ nhập map bằng chuột.
        while not done:
            # [Giải thích dòng gốc 133] Duyệt tất cả sự kiện phát sinh từ cửa sổ `pygame`.
            for event in pygame.event.get():
                # [Giải thích dòng gốc 134] Kiểm tra xem người dùng có đóng cửa sổ hay không.
                if event.type == pygame.QUIT:
                    # [Giải thích dòng gốc 135] Nếu có yêu cầu đóng cửa sổ thì thoát vòng lặp chính.
                    done = True
            # [Giải thích dòng gốc 136] Xóa khung hình cũ bằng nền đen trước khi vẽ lại toàn bộ map.
            scr.fill((0,0,0))
            # [Giải thích dòng gốc 137] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for row in range(self.mapSize):
                # [Giải thích dòng gốc 138] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
                for column in range(self.mapSize):
                    # [Giải thích dòng gốc 139] Mặc định màu của ô là trắng, tương ứng ô trống.
                    color = (255,255,255)
                    # [Giải thích dòng gốc 140] Nếu ô hiện tại là vật cản thì đổi màu.
                    if grid[row][column] == 1:
                        # [Giải thích dòng gốc 141] Đặt vật cản màu đen.
                        color = (0,0,0)
                    # [Giải thích dòng gốc 142] Vẽ nhanh một ô với cách viết gọn một dòng.
                    pygame.draw.rect(scr,color,[(MARGIN + WIDTH) * column + MARGIN,(MARGIN + HEIGHT) * row + MARGIN,WIDTH,HEIGHT])

                    # [Giải thích dòng gốc 144] Mặc định màu của ô là trắng, tương ứng ô trống.
                    color = (255,255,255)
                    # [Giải thích dòng gốc 145] Nếu ô là goal thì sẽ vẽ chồng một hình chữ nhật màu riêng.
                    if (grid[row][column] == 2):
                        # [Giải thích dòng gốc 146] Màu xanh lá để biểu diễn goal/checkpoint.
                        color = (0,255,0)
                        # [Giải thích dòng gốc 147] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
                        pygame.draw.rect(scr,color,[(MARGIN + WIDTH) * column + MARGIN,(MARGIN + HEIGHT) * row + MARGIN,WIDTH,HEIGHT])

            # [Giải thích dòng gốc 149] Giới hạn tốc độ vòng lặp giao diện ở khoảng 50 FPS.
            clock.tick(50)
            # [Giải thích dòng gốc 150] Đẩy toàn bộ nội dung đã vẽ lên màn hình.
            pygame.display.flip()
        # [Giải thích dòng gốc 151] Kết thúc hàm; dòng `return` trống nghĩa là không trả giá trị nào hữu ích.
        return 



    # [Giải thích dòng gốc 155] Kiểm tra xem có thể đi chéo giữa hai ô lân cận mà không cắt góc qua obstacle hay không.
    def goStraight(self,p1,p2):
        # [Giải thích dòng gốc 156] Nếu ô giao giữa hàng của `p1` và cột của `p2` là vật cản thì không cho đi.
        if self.gridMap[p1[0]][p2[1]] == 1: return 0
        # [Giải thích dòng gốc 157] Kiểm tra ô giao còn lại; nếu là vật cản thì cũng cấm đi chéo.
        if self.gridMap[p2[0]][p1[1]] == 1: return 0
        # [Giải thích dòng gốc 158] Nếu cả hai ô chéo góc đều không chặn thì cho phép đi.
        return 1
        


    # [Giải thích dòng gốc 162] Kiểm tra một cặp toạ độ `(u, v)` có nằm trong map và không rơi vào obstacle hay không.
    def validpos(self,u,v):
        # [Giải thích dòng gốc 163] Comment gốc mô tả ý nghĩa của điều kiện kiểm tra tính hợp lệ.
        # (u,v) is in the map and not obstacle
        # [Giải thích dòng gốc 164] Trả về sai nếu toạ độ vượt biên hoặc ô tương ứng là vật cản.
        if (u<0) or (u>self.mapSize-1) or (v<0) or (v>self.mapSize-1) or (self.gridMap[int(u)][int(v)]==1):
            # [Giải thích dòng gốc 165] Mã hoá `False` bằng số 0.
            return 0
        # [Giải thích dòng gốc 166] Nếu cả hai ô chéo góc đều không chặn thì cho phép đi.
        return 1



    # [Giải thích dòng gốc 170] Tính khoảng cách Euclid giữa hai điểm trên lưới.
    def distant(self,x1,y1,x2,y2):
        # [Giải thích dòng gốc 171] Hiệu toạ độ theo trục hàng/x.
        dx = x1-x2
        # [Giải thích dòng gốc 172] Hiệu toạ độ theo trục cột/y.
        dy = y1-y2
        # [Giải thích dòng gốc 173] Áp dụng công thức căn của tổng bình phương hai độ lệch.
        re = math.sqrt(dx*dx + dy*dy)
        # [Giải thích dòng gốc 174] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re


    # [Giải thích dòng gốc 177] Xây dựng ma trận tích lũy 2D để đếm nhanh số vật cản trong hình chữ nhật bất kỳ.
    def buildSumBlock(self):
        # [Giải thích dòng gốc 178] Tạo prefix-sum 2D kích thước lớn hơn map 1 đơn vị ở mỗi chiều để tiện truy vấn biên.
        self.sumBlock = [ [0 for i in range(self.mapSize+1)] for i in range(self.mapSize+1)]
        # [Giải thích dòng gốc 179] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 180] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 181] Cập nhật công thức prefix sum 2D chuẩn từ trái, trên và chéo trên-trái.
                self.sumBlock[i+1][j+1] = self.sumBlock[i+1][j] + self.sumBlock[i][j+1] - self.sumBlock[i][j]
                # [Giải thích dòng gốc 182] Nếu ô gốc là obstacle thì tăng tổng tích lũy lên 1.
                if (self.gridMap[i][j]==1):
                    # [Giải thích dòng gốc 183] Cộng thêm chính ô hiện tại vào tổng vật cản.
                    self.sumBlock[i+1][j+1] += 1
                    


    # [Giải thích dòng gốc 187] Đếm số obstacle trong hình chữ nhật có hai góc là `(x1,y1)` và `(x2,y2)`.
    def countBlock(self,x1,y1,x2,y2):
        # [Giải thích dòng gốc 188] Dịch chỉ số sang hệ 1-based để khớp với prefix sum 2D.
        x1 += 1
        # [Giải thích dòng gốc 189] Dịch tung độ đầu tiên sang hệ 1-based.
        y1 += 1
        # [Giải thích dòng gốc 190] Dịch hoành độ thứ hai sang hệ 1-based.
        x2 += 1
        # [Giải thích dòng gốc 191] Dịch tung độ thứ hai sang hệ 1-based.
        y2 += 1
        # [Giải thích dòng gốc 192] Nếu người gọi truyền góc theo thứ tự ngược thì đảo lại.
        if (x1>x2):
            # [Giải thích dòng gốc 193] Hoán đổi hai đầu mút theo trục x/hàng.
            x1,x2 = x2,x1
        # [Giải thích dòng gốc 194] Nếu thứ tự theo trục y/cột bị ngược thì cũng đảo lại.
        if (y1>y2):
            # [Giải thích dòng gốc 195] Hoán đổi hai đầu mút theo trục y/cột.
            y1,y2 = y2,y1

        # [Giải thích dòng gốc 197] Áp dụng công thức truy vấn prefix sum 2D để lấy số obstacle trong hình chữ nhật.
        re = self.sumBlock[x2][y2] - self.sumBlock[x1-1][y2] - self.sumBlock[x2][y1-1] + self.sumBlock[x1-1][y1-1]
        # [Giải thích dòng gốc 198] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re



    # [Giải thích dòng gốc 202] Ước lượng xem đoạn nối thẳng giữa hai điểm có bị obstacle chen giữa hay không thông qua đếm block trong các vùng liên quan.
    def connectable(self,x1,y1,x2,y2):
        # [Giải thích dòng gốc 203] Biến cờ để ghi nhận hướng tương đối sau khi chuẩn hoá thứ tự hai đầu mút.
        vec = 0
        # [Giải thích dòng gốc 204] Nếu người gọi truyền góc theo thứ tự ngược thì đảo lại.
        if (x1>x2):
            # [Giải thích dòng gốc 205] Hoán đổi hai đầu mút theo trục x/hàng.
            x1,x2 = x2,x1
            # [Giải thích dòng gốc 206] Đảo bit cờ bằng XOR để ghi lại việc đã đổi chiều theo một trục.
            vec = vec^1
        # [Giải thích dòng gốc 207] Nếu thứ tự theo trục y/cột bị ngược thì cũng đảo lại.
        if (y1>y2):
            # [Giải thích dòng gốc 208] Hoán đổi hai đầu mút theo trục y/cột.
            y1,y2 = y2,y1
            # [Giải thích dòng gốc 209] Đảo bit cờ bằng XOR để ghi lại việc đã đổi chiều theo một trục.
            vec = vec^1

        # [Giải thích dòng gốc 211] Đếm tổng số obstacle trong hình chữ nhật bao quanh hai điểm.
        tot = self.countBlock(x1,y1,x2,y2)
        # [Giải thích dòng gốc 212] Tính kích thước vùng nửa theo trục x dùng để trừ bớt hai góc không liên quan.
        dx = (x2-x1+1)//2 - 1
        # [Giải thích dòng gốc 213] Tính kích thước vùng nửa theo trục y tương tự.
        dy = (y2-y1+1)//2 - 1
        # [Giải thích dòng gốc 214] Khởi tạo số obstacle vùng cần loại khỏi một góc.
        t1 = 0
        # [Giải thích dòng gốc 215] Khởi tạo số obstacle vùng cần loại khỏi góc còn lại.
        t2 = 0
        # [Giải thích dòng gốc 216] Chỉ khi hình chữ nhật đủ lớn theo cả hai chiều thì mới tách hai góc để hiệu chỉnh.
        if dx>0 and dy>0:
            # [Giải thích dòng gốc 217] Phân nhánh theo hướng chuẩn hoá của đoạn nối giữa hai điểm.
            if (vec==1):
                # [Giải thích dòng gốc 218] Đếm obstacle trong một góc nhỏ đầu tiên để loại khỏi tổng.
                t1 = self.countBlock(x1,y1,x1+dx-1,y1+dy-1)
                # [Giải thích dòng gốc 219] Đếm obstacle trong góc nhỏ đối diện để loại khỏi tổng.
                t2 = self.countBlock(x2-dx+1,y2-dy+1,x2,y2)
            # [Giải thích dòng gốc 220] Nhánh còn lại khi các điều kiện trước không thỏa.
            else:
                # [Giải thích dòng gốc 221] Trường hợp hướng ngược, lấy góc nhỏ ở phía tương ứng thứ nhất.
                t1 = self.countBlock(x2-dx+1,y1,x2,y1+dy-1)
                # [Giải thích dòng gốc 222] Trường hợp hướng ngược, lấy góc nhỏ ở phía tương ứng thứ hai.
                t2 = self.countBlock(x1,y2-dy+1,x1+dx-1,y2)
        # [Giải thích dòng gốc 223] Số obstacle còn lại sau khi trừ hai vùng góc sẽ dùng làm chỉ báo cản trở kết nối thẳng.
        cnt = tot - t1 - t2
        # [Giải thích dòng gốc 224] Trả về số obstacle còn vướng; bằng 0 nghĩa là đoạn khá sạch để nối.
        return cnt


    # [Giải thích dòng gốc 227] Thêm hoặc cập nhật một cạnh giữa hai goal/component nếu hai vị trí biên `posA`, `posB` thuộc hai owner khác nhau.
    def add_edge(self, posA, posB):

        # [Giải thích dòng gốc 229] Lấy owner của điểm thứ nhất; `posA[0]` cho biết đang tra ở lớp 0 hay lớp 1 của cấu trúc owner.
        u1 = self.owner[posA[0]][posA[1]][posA[2]]
        # [Giải thích dòng gốc 230] Lấy owner của điểm thứ hai theo cách tương tự.
        u2 = self.owner[posB[0]][posB[1]][posB[2]]
        # [Giải thích dòng gốc 231] Nếu hai điểm cùng thuộc một goal/component thì không tạo cạnh mới.
        if u1 == u2 :return
        # [Giải thích dòng gốc 232] Nếu một trong hai điểm chưa có owner hợp lệ thì bỏ qua.
        if (u1==-1 or u2==-1): return

        # [Giải thích dòng gốc 234] Ước lượng chi phí nối hai component bằng tổng arrival time của hai điểm chạm biên.
        totdis = self.dista[posA[1]][posA[2]] + self.dista[posB[1]][posB[2]]

        # [Giải thích dòng gốc 236] Chỉ cập nhật nếu cạnh mới tốt hơn cạnh cũ đang lưu.
        if self.adj[u1][u2] > totdis:
            # [Giải thích dòng gốc 237] Ghi trọng số theo chiều từ `u1` sang `u2`.
            self.adj[u1][u2] = totdis
            # [Giải thích dòng gốc 238] Đồ thị vô hướng nên gán đối xứng cho chiều ngược lại.
            self.adj[u2][u1] = totdis
            # [Giải thích dòng gốc 239] Lưu lại cặp điểm giao nhau tạo nên cạnh tốt nhất hiện tại.
            self.inters[u1][u2] = (posA,posB)
            # [Giải thích dòng gốc 240] Lưu bản đối xứng cho chiều ngược.
            self.inters[u2][u1] = (posB,posA)



    # [Giải thích dòng gốc 244] Sinh danh sách láng giềng hợp lệ từ một ô, theo số hướng cần xét là `dir`.
    def spread(self,pos,dir):

        # [Giải thích dòng gốc 246] Danh sách kết quả chứa toàn bộ các điểm của quỹ đạo cuối.
        re = []
        # [Giải thích dòng gốc 247] Duyệt qua `dir` hướng đầu tiên trong bảng hướng; comment ghi 4 hướng nhưng thực tế nếu truyền 8 thì sẽ đi đủ 8 hướng.
        for h in range(0,dir): # 4 direction
            # [Giải thích dòng gốc 248] Tính hàng của ô láng giềng theo hướng thứ `h`.
            u2 = pos[0] + self.tx[h]
            # [Giải thích dòng gốc 249] Tính cột của ô láng giềng theo hướng thứ `h`.
            v2 = pos[1] + self.ty[h]
            # [Giải thích dòng gốc 250] Nếu ô láng giềng ra ngoài map hoặc là obstacle thì bỏ qua.
            if (self.validpos(u2,v2)==0): continue
            # [Giải thích dòng gốc 251] Nếu di chuyển theo hướng đó gây cắt góc qua vật cản thì cũng bỏ qua.
            if self.goStraight(pos,(u2,v2)) == 0: continue

            # [Giải thích dòng gốc 253] Thêm láng giềng hợp lệ cùng chỉ số hướng vào danh sách kết quả.
            re.append( ((u2,v2),h) )

        # [Giải thích dòng gốc 255] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re
            


    # [Giải thích dòng gốc 259] Xây dựng `forest`/wavefront cơ bản: các goal lan đồng thời để gán owner, arrival time và tạo cạnh giao nhau giữa các vùng.
    def buildForest(self):

        # [Giải thích dòng gốc 261] Khởi tạo trường khoảng cách lớn cho mọi ô; giá trị lớn đóng vai trò vô cùng.
        self.dista = [ [self.mapSize*self.mapSize for i in range(self.mapSize)] for i in range(self.mapSize)]
        # [Giải thích dòng gốc 262] Khởi tạo mảng trace 3D cho 2 lớp truy vết; mỗi ô ban đầu chưa có cha nên là `(-1,-1,-1)`.
        self.trace = [ [ [ (-1,-1,-1) for i in range(self.mapSize)] for i in range(self.mapSize)] for i in range(2)]
        # [Giải thích dòng gốc 263] Khởi tạo mảng owner 3D cho 2 lớp; `-1` nghĩa là ô chưa được goal nào chiếm.
        self.owner = [ [ [-1 for i in range(self.mapSize)] for i in range(self.mapSize)] for i in range(2)]
        # [Giải thích dòng gốc 264] Khởi tạo ma trận kề giữa các goal với trọng số vô cùng.
        self.adj = [ [self.mapSize*self.mapSize for i in range(self.npos)] for i in range(self.npos)]
        # [Giải thích dòng gốc 265] Khởi tạo ma trận lưu các cặp điểm giao/tiếp xúc tốt nhất giữa từng cặp goal.
        self.inters = [ [ ((-1,-1,-1),(-1,-1,-1)) for i in range(self.npos) ] for i in range(self.npos)]
        # [Giải thích dòng gốc 266] Ma trận đánh dấu ô đã `freeze` hay chưa; freeze nghĩa là đã chốt giá trị theo kiểu Fast Marching/Dijkstra.
        self.froze = [ [ 0 for i in range(self.mapSize) ] for i in range(self.mapSize)]

        # [Giải thích dòng gốc 268] Tạo hàng đợi ưu tiên để luôn lấy ô có arrival time nhỏ nhất trước.
        pq = queue.PriorityQueue()
        # [Giải thích dòng gốc 269] Khởi tạo một nguồn lan cho từng goal.
        for i in range(self.npos): 
            # [Giải thích dòng gốc 270] Lấy toạ độ goal thứ `i`.
            pos = self.deslist[i]
            # [Giải thích dòng gốc 271] Đưa goal vào hàng đợi với chi phí ban đầu bằng 0.
            pq.put((0,pos))
            # [Giải thích dòng gốc 272] Arrival time tại goal luôn bằng 0.
            self.dista[pos[0]][pos[1]] = 0
            # [Giải thích dòng gốc 273] Goal là nguồn nên không có cha truy vết.
            self.trace[0][pos[0]][pos[1]] = (-1,-1,-1)
            # [Giải thích dòng gốc 274] Đánh dấu goal tự sở hữu chính ô của nó.
            self.owner[0][pos[0]][pos[1]] = i

        # [Giải thích dòng gốc 276] Bắt đầu vòng lặp `while`; khối lệnh bên dưới sẽ lặp lại khi điều kiện còn đúng.
        while not pq.empty():
            # [Giải thích dòng gốc 277] Lấy ô có arrival time nhỏ nhất hiện tại ra xử lý.
            item = pq.get()
            # [Giải thích dòng gốc 278] Sinh các láng giềng 4 hướng của ô đang xét để cập nhật sóng lan.
            nxt = self.spread(item[1],4) # 4 direction
            # [Giải thích dòng gốc 279] Nếu ô này đã bị chốt từ trước thì bỏ qua phần tử trùng trong priority queue.
            if self.froze[item[1][0]][item[1][1]] == 1: continue
            # [Giải thích dòng gốc 280] Đánh dấu ô hiện tại đã được chốt giá trị khoảng cách.
            self.froze[item[1][0]][item[1][1]] = 1

            # [Giải thích dòng gốc 282] Duyệt từng láng giềng hợp lệ cần thử cập nhật.
            for (u2,v2),h in nxt:

                # [Giải thích dòng gốc 284] Chỉ cập nhật ô láng giềng nếu nó chưa bị freeze.
                if self.froze[u2][v2] == 0:
                    # [Giải thích dòng gốc 285] Khởi tạo giá trị tốt nhất theo trục x/hàng bằng vô cùng.
                    dx = self.mapSize*self.mapSize
                    # [Giải thích dòng gốc 286] Nếu ô phía trên tồn tại và hợp lệ thì xét nó làm ứng viên.
                    if (self.validpos(u2-1,v2)!=0):
                        # [Giải thích dòng gốc 287] Kiểm tra thừa lần nữa rằng ô đó không phải obstacle.
                        if (self.gridMap[u2-1][v2]!=1):
                            # [Giải thích dòng gốc 288] Lấy arrival time nhỏ hơn giữa các láng giềng theo trục dọc.
                            dx = min(dx,self.dista[u2-1][v2])
                    # [Giải thích dòng gốc 289] Xét tiếp ô phía dưới nếu hợp lệ.
                    if (self.validpos(u2+1,v2)!=0):
                        # [Giải thích dòng gốc 290] Kiểm tra lại trạng thái obstacle của ô dưới.
                        if (self.gridMap[u2+1][v2]!=1):
                            # [Giải thích dòng gốc 291] Cập nhật giá trị nhỏ nhất theo trục dọc.
                            dx = min(dx,self.dista[u2+1][v2])

                    # [Giải thích dòng gốc 293] Khởi tạo giá trị tốt nhất theo trục y/cột bằng vô cùng.
                    dy = self.mapSize*self.mapSize
                    # [Giải thích dòng gốc 294] Xét ô bên trái nếu hợp lệ.
                    if (self.validpos(u2,v2-1)!=0):
                        # [Giải thích dòng gốc 295] Đảm bảo ô bên trái không phải obstacle.
                        if (self.gridMap[u2][v2-1]!=1):
                            # [Giải thích dòng gốc 296] Cập nhật arrival time tốt nhất theo trục ngang.
                            dy = min(dy,self.dista[u2][v2-1])
                    # [Giải thích dòng gốc 297] Xét ô bên phải nếu hợp lệ.
                    if (self.validpos(u2,v2+1)!=0):
                        # [Giải thích dòng gốc 298] Đảm bảo ô bên phải không phải obstacle.
                        if (self.gridMap[u2][v2+1]!=1):
                            # [Giải thích dòng gốc 299] Cập nhật arrival time tốt nhất theo trục ngang.
                            dy = min(dy,self.dista[u2][v2+1])

                    # [Giải thích dòng gốc 301] Biến tạm để lưu arrival time mới tính cho ô `(u2, v2)`.
                    dis = 0
                    # [Giải thích dòng gốc 302] Lấy giá trị hiện đang có của ô này để so sánh/cập nhật.
                    T = self.dista[u2][v2]
                    # [Giải thích dòng gốc 303] Trường hợp cả hai hướng đều có láng giềng tốt hơn; dùng nghiệm rời rạc gần đúng của Eikonal.
                    if (T>dx and T>dy):
                        # [Giải thích dòng gốc 304] Công thức cập nhật Fast Marching 2D khi cả hai phía cùng tham gia vào front.
                        dis = (math.sqrt(-(dx*dx)+2*dx*dy-(dy*dy)+2) +dx +dy)/2
                    # [Giải thích dòng gốc 305] Nếu chỉ hướng x/hàng mang thông tin tốt hơn thì cập nhật một chiều.
                    elif (T>dx and T<=dy):
                        # [Giải thích dòng gốc 306] Arrival time mới bằng giá trị theo x cộng chi phí đi thêm 1 ô.
                        dis = dx + 1
                    # [Giải thích dòng gốc 307] Nếu chỉ hướng y/cột mang thông tin tốt hơn thì cập nhật một chiều còn lại.
                    elif (T<=dx and T>dy):
                        # [Giải thích dòng gốc 308] Arrival time mới bằng giá trị theo y cộng thêm 1.
                        dis = dy + 1

                    # [Giải thích dòng gốc 310] Nếu giá trị mới tốt hơn giá trị cũ thì chấp nhận cập nhật.
                    if (self.dista[u2][v2] > dis):
                        # [Giải thích dòng gốc 311] Ghi arrival time mới cho ô láng giềng.
                        self.dista[u2][v2] = dis
                        # [Giải thích dòng gốc 312] Đưa ô vừa được cải thiện vào hàng đợi ưu tiên để xử lý tiếp.
                        pq.put( ( dis , (u2,v2)) )
                        # [Giải thích dòng gốc 313] Lấy các láng giềng gần để tìm cha truy vết và owner; comment ghi 8-direction nhưng ở đây thực tế truyền 4.
                        nxt2 = self.spread((u2,v2),4) # 8-direction
                        # [Giải thích dòng gốc 314] Biến lưu khoảng cách nhỏ nhất tìm thấy ở các ô kề đã có owner.
                        nho1 = self.mapSize*self.mapSize
                        # [Giải thích dòng gốc 315] Biến lưu toạ độ của láng giềng tốt nhất làm cha truy vết.
                        nho2 = (-1,-1)
                        # [Giải thích dòng gốc 316] Duyệt các láng giềng kề cận của ô vừa cập nhật.
                        for (u3,v3),h in nxt2:
                            # [Giải thích dòng gốc 317] Chọn láng giềng đã có owner và có arrival time nhỏ nhất để làm cha.
                            if (self.owner[0][u3][v3]!=-1 and self.dista[u3][v3]<nho1):
                                # [Giải thích dòng gốc 318] Cập nhật giá trị nhỏ nhất tạm thời.
                                nho1 = self.dista[u3][v3]
                                # [Giải thích dòng gốc 319] Lưu toạ độ cha truy vết tốt nhất.
                                nho2 = (u3,v3)
                        # [Giải thích dòng gốc 320] Ghi cha của ô trong lớp trace 0, đồng thời nhớ rằng cha nằm ở lớp 0.
                        self.trace[0][u2][v2] = (0,nho2[0],nho2[1])
                        # [Giải thích dòng gốc 321] Gán owner của ô mới theo owner của cha tốt nhất.
                        self.owner[0][u2][v2] = self.owner[0][nho2[0]][nho2[1]]
                        # [Giải thích dòng gốc 322] Đã cập nhật xong ô hiện tại thì chuyển sang láng giềng tiếp theo.
                        continue

        # [Giải thích dòng gốc 324] Comment gốc đánh dấu đoạn sau dùng để thêm các cạnh giữa những vùng đã chạm nhau.
        #add_edge         
        # [Giải thích dòng gốc 325] Duyệt toàn bộ các ô trong map để tìm biên tiếp giáp giữa các owner khác nhau.
        for u in range(self.mapSize):
            # [Giải thích dòng gốc 326] Duyệt cột tương ứng của từng hàng.
            for v in range(self.mapSize):
                # [Giải thích dòng gốc 327] Bỏ qua các ô có owner bằng 0; đây là một quyết định cài đặt hơi lạ nhưng không làm hỏng ý tưởng chung.
                if self.owner[0][u][v] == 0: continue
                # [Giải thích dòng gốc 328] Xét 4 láng giềng của ô hiện tại để kiểm tra khả năng tiếp giáp vùng khác.
                nxt = self.spread((u,v),4)
                # [Giải thích dòng gốc 329] Duyệt từng láng giềng để thử tạo cạnh.
                for (u2,v2),h in nxt:
                    # [Giải thích dòng gốc 330] Nếu hai ô thuộc hai owner khác nhau thì `add_edge` sẽ cập nhật cạnh goal-goal tương ứng.
                    self.add_edge((0,u,v),(0,u2,v2))


    # [Giải thích dòng gốc 333] Pha FMF nâng cao: sau khi có forest cơ bản, cho từng component lan tiếp bên trong vùng của chính nó để tạo thêm điểm chạm/cạnh mới.
    def buildForestAdvanced(self): # require buildForest before

        # [Giải thích dòng gốc 335] Ma trận cờ cho biết ở vòng hiện tại component được phép lan vào ô nào.
        self.allow = [ [0 for i in range(self.mapSize)] for i in range(self.mapSize)]
        # [Giải thích dòng gốc 336] Ma trận tạm để sao lưu `dista` cũ của các ô thuộc component đang xét.
        self.mirrow = [ [0 for i in range(self.mapSize)] for i in range(self.mapSize)]
        # [Giải thích dòng gốc 337] Danh sách các ô thuộc mỗi owner/component ở lớp 0.
        self.hold = [ [] for i in range(self.npos) ]

        # [Giải thích dòng gốc 339] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 340] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 341] Lấy owner lớp 0 của ô `(i, j)`.
                own = self.owner[0][i][j]
                # [Giải thích dòng gốc 342] Nếu ô đã được một goal chiếm giữ thì thêm nó vào component tương ứng.
                if (own!=-1):
                    # [Giải thích dòng gốc 343] Gom ô `(i, j)` vào danh sách ô của owner `own`.
                    self.hold[own].append((i,j))

        # [Giải thích dòng gốc 345] Lần lượt kích hoạt từng component để thực hiện pha firework nâng cao.
        for comp in range(self.npos):
            # [Giải thích dòng gốc 346] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
            pq = queue.PriorityQueue()
            # [Giải thích dòng gốc 347] Duyệt mọi ô đang thuộc component hiện tại.
            for pos in self.hold[comp]:
                # [Giải thích dòng gốc 348] Đánh dấu rằng component này được phép lan trên vùng của chính nó.
                self.allow[pos[0]][pos[1]] = 1
                # [Giải thích dòng gốc 349] Lưu lại arrival time gốc của ô để sau này khôi phục.
                self.mirrow[pos[0]][pos[1]] = self.dista[pos[0]][pos[1]]
                # [Giải thích dòng gốc 350] Tạm đặt khoảng cách của vùng này về vô cùng để chạy một front mới bên trong component.
                self.dista[pos[0]][pos[1]] = self.mapSize*self.mapSize
                # [Giải thích dòng gốc 351] Bỏ đóng băng các ô của component để front mới có thể lan lại qua chúng.
                self.froze[pos[0]][pos[1]] = 0

                # [Giải thích dòng gốc 353] Xét 8 láng giềng của ô biên hiện tại để lấy các nguồn tiếp xúc bên ngoài component.
                nxt = self.spread(pos,8)
                # [Giải thích dòng gốc 354] Duyệt từng láng giềng để thử tạo cạnh.
                for (u2,v2),h in nxt:
                    # [Giải thích dòng gốc 355] Chỉ quan tâm tới láng giềng không thuộc cùng component hiện tại.
                    if self.owner[0][u2][v2] != comp:
                        # [Giải thích dòng gốc 356] Đưa láng giềng ngoài component vào priority queue như mồi cho front mới.
                        pq.put( (self.dista[u2][v2],(0,u2,v2)) )
                        # [Giải thích dòng gốc 357] Cho phép láng giềng đó được xử lý lại trong pha lan nâng cao.
                        self.froze[u2][v2] = 0
            
            # [Giải thích dòng gốc 359] Bắt đầu vòng lặp `while`; khối lệnh bên dưới sẽ lặp lại khi điều kiện còn đúng.
            while not pq.empty():
                # [Giải thích dòng gốc 360] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                item = pq.get()
                # [Giải thích dòng gốc 361] Từ phần tử lấy ra, xét 4 láng giềng để lan tiếp.
                nxt = self.spread((item[1][1],item[1][2]),4)
                # [Giải thích dòng gốc 362] Nếu ô đã được chốt ở pha này thì bỏ qua bản sao trùng trong queue.
                if self.froze[item[1][1]][item[1][2]] == 1: continue
                # [Giải thích dòng gốc 363] Chốt ô hiện tại trong pha lan nâng cao.
                self.froze[item[1][1]][item[1][2]] = 1

                # [Giải thích dòng gốc 365] Duyệt từng láng giềng để thử tạo cạnh.
                for (u2,v2),h in nxt:
                    # [Giải thích dòng gốc 366] Chỉ cho lan vào những ô thuộc vùng component hiện tại.
                    if self.allow[u2][v2] == 0: continue

                    # [Giải thích dòng gốc 368] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                    if self.froze[u2][v2] == 0:
                        # [Giải thích dòng gốc 369] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                        dx = self.mapSize*self.mapSize
                        # [Giải thích dòng gốc 370] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                        if (self.validpos(u2-1,v2)!=0):
                            # [Giải thích dòng gốc 371] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                            if (self.gridMap[u2-1][v2]!=1):
                                # [Giải thích dòng gốc 372] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                                dx = min(dx,self.dista[u2-1][v2])
                        # [Giải thích dòng gốc 373] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                        if (self.validpos(u2+1,v2)!=0):
                            # [Giải thích dòng gốc 374] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                            if (self.gridMap[u2+1][v2]!=1):
                                # [Giải thích dòng gốc 375] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                                dx = min(dx,self.dista[u2+1][v2])

                        # [Giải thích dòng gốc 377] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                        dy = self.mapSize*self.mapSize
                        # [Giải thích dòng gốc 378] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                        if (self.validpos(u2,v2-1)!=0):
                            # [Giải thích dòng gốc 379] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                            if (self.gridMap[u2][v2-1]!=1):
                                # [Giải thích dòng gốc 380] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                                dy = min(dy,self.dista[u2][v2-1])
                        # [Giải thích dòng gốc 381] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                        if (self.validpos(u2,v2+1)!=0):
                            # [Giải thích dòng gốc 382] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                            if (self.gridMap[u2][v2+1]!=1):
                                # [Giải thích dòng gốc 383] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                                dy = min(dy,self.dista[u2][v2+1])  

                        # [Giải thích dòng gốc 385] Khởi tạo biến bằng 0.
                        dis = 0
                        # [Giải thích dòng gốc 386] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                        T = self.dista[u2][v2]
                        # [Giải thích dòng gốc 387] Comment debug cũ, dùng để in hai giá trị lân cận theo trục x và y.
                        #print(dx," ",dy)
                        # [Giải thích dòng gốc 388] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                        if (T>dx and T>dy):
                            # [Giải thích dòng gốc 389] Tính biểu thức dưới căn cho công thức cập nhật Eikonal 2D.
                            val = -(dx*dx)+2*dx*dy-(dy*dy)+2
                            # [Giải thích dòng gốc 390] Chỉ lấy căn khi biểu thức dưới căn không âm.
                            if (val>=0):
                                # [Giải thích dòng gốc 391] Nếu hợp lệ thì dùng công thức hai phía chuẩn.
                                dis = (math.sqrt(val) +dx +dy)/2
                            # [Giải thích dòng gốc 392] Nếu biểu thức dưới căn âm do sai số/hình học thì dùng phương án an toàn.
                            else:
                                # [Giải thích dòng gốc 393] Fallback: lấy hướng tốt hơn rồi cộng một bước.
                                dis = min(dx,dy)+1
                        # [Giải thích dòng gốc 394] Nhánh điều kiện phụ được xét khi các nhánh trước không thỏa.
                        elif (T>dx and T<=dy):
                            # [Giải thích dòng gốc 395] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                            dis = dx + 1
                        # [Giải thích dòng gốc 396] Nhánh điều kiện phụ được xét khi các nhánh trước không thỏa.
                        elif (T<=dx and T>dy):
                            # [Giải thích dòng gốc 397] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                            dis = dy + 1

                        # [Giải thích dòng gốc 399] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                        if (self.dista[u2][v2] > dis):
                            # [Giải thích dòng gốc 400] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                            self.dista[u2][v2] = dis
                            # [Giải thích dòng gốc 401] Đưa ô vừa cải thiện vào queue, nhưng gắn nhãn lớp 1 để phân biệt trace/owner của pha nâng cao.
                            pq.put( ( dis , (1,u2,v2)) )
                            # [Giải thích dòng gốc 402] Xét 8 láng giềng để tìm cha truy vết tốt nhất cho pha 2 này.
                            nxt2 = self.spread((u2,v2),8) # 8-direction
                            # [Giải thích dòng gốc 403] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                            nho1 = self.mapSize*self.mapSize
                            # [Giải thích dòng gốc 404] Lưu cha tốt nhất dưới dạng `(layer, row, col)`.
                            nho2 = (-1,-1,-1)
                            # [Giải thích dòng gốc 405] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
                            for (u3,v3),h in nxt2:
                                # [Giải thích dòng gốc 406] Ưu tiên láng giềng lớp 0 nằm ngoài vùng `allow` nhưng có owner hợp lệ và distance tốt.
                                if (self.owner[0][u3][v3]!=-1 and self.allow[u3][v3]==0 and self.dista[u3][v3]<nho1):
                                    # [Giải thích dòng gốc 407] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                                    nho1 = self.dista[u3][v3]
                                    # [Giải thích dòng gốc 408] Ghi nhận cha tốt nhất đang nằm ở lớp 0.
                                    nho2 = (0,u3,v3)
                                # [Giải thích dòng gốc 409] Hoặc chọn láng giềng lớp 1 đã được tạo trong pha này nếu nó tốt hơn.
                                if (self.owner[1][u3][v3]!=-1 and self.allow[u3][v3]==1 and self.dista[u3][v3]<nho1):         
                                    # [Giải thích dòng gốc 410] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                                    nho1 = self.dista[u3][v3]
                                    # [Giải thích dòng gốc 411] Ghi nhận cha tốt nhất ở lớp 1.
                                    nho2 = (1,u3,v3)
                            # [Giải thích dòng gốc 412] Lưu thông tin cha truy vết cho ô hiện tại trong lớp 1.
                            self.trace[1][u2][v2] = nho2
                            # [Giải thích dòng gốc 413] Kế thừa owner từ cha tốt nhất, bất kể cha ở lớp 0 hay lớp 1.
                            self.owner[1][u2][v2] = self.owner[nho2[0]][nho2[1]][nho2[2]]
                            # [Giải thích dòng gốc 414] Bỏ phần xử lý còn lại của vòng lặp hiện tại và chuyển sang lượt kế tiếp.
                            continue

                    # [Giải thích dòng gốc 416] Đoạn code dự định thêm cạnh trực tiếp khi gặp owner lớp 1 đã bị comment lại.
                    #if (self.owner[1][u2][v2]!=-1):
                    # [Giải thích dòng gốc 417] Lời gọi cũ đến `add_edge` đã bị vô hiệu hóa.
                    #    self.add_edge((item[1]),(1,u2,v2),dis)
                    # [Giải thích dòng gốc 418] Đoạn code dự định thêm cạnh khi gặp owner lớp 0 cũng bị comment.
                    #if (self.owner[0][u2][v2]!=-1):
                    # [Giải thích dòng gốc 419] Lời gọi tương ứng cũng không còn dùng.
                    #    self.add_edge((item[1]),(0,u2,v2),dis)

                # [Giải thích dòng gốc 421] Comment đánh dấu rằng sau vòng lan sẽ thực hiện bước thêm cạnh.
                #add_edge         
            # [Giải thích dòng gốc 422] Duyệt mọi ô đang thuộc component hiện tại.
            for pos in self.hold[comp]:
                # [Giải thích dòng gốc 423] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                nxt = self.spread(pos,4)
                # [Giải thích dòng gốc 424] Duyệt từng láng giềng để thử tạo cạnh.
                for (u2,v2),h in nxt:
                    # [Giải thích dòng gốc 425] Nếu láng giềng thuộc lớp 0 bên ngoài vùng hiện tại thì nối lớp 1 của component sang lớp 0 đó.
                    if (self.owner[0][u2][v2]!=-1 and self.allow[u2][v2]==0):
                        # [Giải thích dòng gốc 426] Cập nhật cạnh nhờ điểm tiếp giáp giữa wavefront nâng cao và lớp gốc.
                        self.add_edge((1,pos[0],pos[1]),(0,u2,v2))
                    # [Giải thích dòng gốc 427] Nếu láng giềng nằm trong lớp 1 cùng vùng allow thì cũng có thể tạo cạnh thông qua front mới.
                    if (self.owner[1][u2][v2]!=-1 and self.allow[u2][v2]==1):
                        # [Giải thích dòng gốc 428] Thêm/cập nhật cạnh dựa trên hai điểm trong pha nâng cao.
                        self.add_edge((1,pos[0],pos[1]),(1,u2,v2))

            # [Giải thích dòng gốc 430] Duyệt mọi ô đang thuộc component hiện tại.
            for pos in self.hold[comp]:
                # [Giải thích dòng gốc 431] Khôi phục cờ `allow` của ô về 0 để chuẩn bị cho component tiếp theo.
                self.allow[pos[0]][pos[1]] = 0
                # [Giải thích dòng gốc 432] Khôi phục arrival time gốc đã lưu tạm trong `mirrow`.
                self.dista[pos[0]][pos[1]] = self.mirrow[pos[0]][pos[1]]
        
            


    # [Giải thích dòng gốc 437] Dựng lại đường đi thô giữa hai goal `u1`, `u2` từ cặp điểm giao nhau tốt nhất đã lưu.
    def getAdj(self,u1,u2): # path from u1 to u2
        # [Giải thích dòng gốc 438] Danh sách kết quả chứa toàn bộ các điểm của quỹ đạo cuối.
        re = []
        # [Giải thích dòng gốc 439] Lấy ra hai điểm tiếp xúc biên tạo nên cạnh tốt nhất giữa hai goal.
        p1,p2 = self.inters[u1][u2]
        # [Giải thích dòng gốc 440] Chuyển điểm giao đầu tiên sang list để tiện cập nhật khi lần ngược trace.
        p1 = list(p1)
        # [Giải thích dòng gốc 441] Chuyển điểm giao thứ hai sang list tương tự.
        p2 = list(p2)
        # [Giải thích dòng gốc 442] Comment debug cũ để in hai điểm giao.
        #print(p1," ",p2)
        # [Giải thích dòng gốc 443] Comment debug cũ để kiểm tra owner của các điểm giao.
        #print(self.owner[p1[0]][p1[1]]," ",self.owner[p2[0]][p2[1]])
        # [Giải thích dòng gốc 444] Comment debug cũ để kiểm tra trace của các điểm giao.
        #print(self.trace[p1[0]][p1[1]]," ",self.trace[p2[0]][p2[1]])
        # [Giải thích dòng gốc 445] Lấy cha đầu tiên của `p1` trong cấu trúc trace tương ứng lớp của nó.
        tra = self.trace[p1[0]][p1[1]][p1[2]]
        # [Giải thích dòng gốc 446] Thêm toạ độ 2D của `p1` vào danh sách đường đi.
        re.append(p1[1:3].copy())
        # [Giải thích dòng gốc 447] Lần ngược cho đến khi gặp nguồn gốc có cha `-1`.
        while (tra[0] != -1):
            # [Giải thích dòng gốc 448] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
            re.append(p1[1:3].copy())
            # [Giải thích dòng gốc 449] Nhảy sang cha hiện tại.
            p1 = list(tra)
            # [Giải thích dòng gốc 450] Tiếp tục lấy cha của điểm mới.
            tra = self.trace[p1[0]][p1[1]][p1[2]]
        # [Giải thích dòng gốc 451] Thêm toạ độ 2D của `p1` vào danh sách đường đi.
        re.append(p1[1:3].copy())
        # [Giải thích dòng gốc 452] Đảo nửa đầu đường đi để nó đi từ goal nguồn ra đến điểm giao.
        re.reverse()

        # [Giải thích dòng gốc 454] Lấy cha của phía `p2` để ghép nửa sau của đường đi.
        tra = self.trace[p2[0]][p2[1]][p2[2]]
        # [Giải thích dòng gốc 455] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
        re.append(p2[1:3].copy())
        # [Giải thích dòng gốc 456] Lần ngược cho đến khi gặp nguồn gốc có cha `-1`.
        while (tra[0] != -1):
            # [Giải thích dòng gốc 457] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
            re.append(p2[1:3].copy())
            # [Giải thích dòng gốc 458] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
            p2 = list(tra)
            # [Giải thích dòng gốc 459] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
            tra = self.trace[p2[0]][p2[1]][p2[2]]  
        # [Giải thích dòng gốc 460] Dòng lệnh này tiếp tục xử lý logic của hàm hiện tại.
        re.append(p2[1:3].copy())
        # [Giải thích dòng gốc 461] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re



    # [Giải thích dòng gốc 465] Rút gọn đường đi thô `pos` thành polyline ngắn hơn và tính chiều dài Euclid của polyline đó.
    def getTrace(self,pos):

        # [Giải thích dòng gốc 467] Danh sách kết quả chứa toàn bộ các điểm của quỹ đạo cuối.
        re = []
        # [Giải thích dòng gốc 468] Biến tích lũy tổng độ dài đường rút gọn.
        dis = 0
        # [Giải thích dòng gốc 469] Điểm neo hiện tại, ban đầu là điểm đầu tiên của đường thô.
        cur = list(pos[0])
        # [Giải thích dòng gốc 470] Duyệt từng điểm trên đường thô để kiểm tra khả năng nối thẳng.
        for i in range(len(pos)):
            # [Giải thích dòng gốc 471] Nếu từ `cur` đến điểm đang xét không còn nối thẳng sạch được thì chốt đoạn trước đó.
            if self.connectable(cur[0],cur[1],pos[i][0],pos[i][1]) !=0 :
                # [Giải thích dòng gốc 472] Cộng chiều dài đoạn từ điểm neo đến điểm trước đó.
                dis += self.distant(cur[0],cur[1],pos[i-1][0],pos[i-1][1])
                # [Giải thích dòng gốc 473] Lưu lại điểm neo hiện tại như một đỉnh của polyline rút gọn.
                re.append(cur.copy())
                # [Giải thích dòng gốc 474] Dời điểm neo sang điểm hợp lệ cuối cùng trước khi gặp cản trở.
                cur = list(pos[i-1])
        
        # [Giải thích dòng gốc 476] Sau vòng lặp, cộng nốt đoạn cuối cùng từ điểm neo tới đích.
        dis += self.distant(cur[0],cur[1],pos[-1][0],pos[-1][1])
        # [Giải thích dòng gốc 477] Thêm điểm neo cuối cùng vào polyline.
        re.append(cur.copy())
        # [Giải thích dòng gốc 478] Thêm điểm đích cuối cùng.
        re.append(list(pos[-1]).copy())

        # [Giải thích dòng gốc 480] Trả về cả polyline đã rút gọn và tổng độ dài của nó.
        return re,dis



    # [Giải thích dòng gốc 484] Dựng đường đi ngắn gọn cho từng cặp goal và cập nhật lại trọng số `adj` theo đường rút gọn đó.
    def twoPointTracing(self):
        # [Giải thích dòng gốc 485] Khởi tạo ma trận lưu đường đi chi tiết giữa từng cặp goal.
        self.pathTrace = [ [ [0] for i in range(self.npos)] for j in range(self.npos)]
        # [Giải thích dòng gốc 486] Xây prefix-sum vật cản để `connectable()` truy vấn nhanh khi rút gọn đường.
        self.buildSumBlock()

        # [Giải thích dòng gốc 488] Lưu số goal vào biến cục bộ cho gọn khi viết vòng lặp.
        n = self.npos
        # [Giải thích dòng gốc 489] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for u in range(n):
            # [Giải thích dòng gốc 490] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for v in range(n):
                # [Giải thích dòng gốc 491] Nếu hai goal chưa có cặp điểm giao hợp lệ thì bỏ qua cặp này.
                if (self.inters[u][v][0][0]==-1): continue

                # [Giải thích dòng gốc 493] Dựng đường thô giữa hai goal từ trace/inters.
                t1 = self.getAdj(u,v)
                # [Giải thích dòng gốc 494] Rút gọn đường thô và đồng thời lấy chiều dài thực của đường rút gọn.
                t2,dis = self.getTrace(t1)

                # [Giải thích dòng gốc 496] Comment debug để in cặp goal đang được truy vết.
                #print("Get trace fron ",u," to ",v)
                # [Giải thích dòng gốc 497] Comment debug để in đường thô trước khi rút gọn.
                #print("Path :",t1)
                # [Giải thích dòng gốc 498] Comment debug để in đường sau khi rút gọn.
                #print("Short:",t2)

                # [Giải thích dòng gốc 500] Lưu polyline rút gọn của cặp `(u, v)`.
                self.pathTrace[u][v] = t2.copy()
                # [Giải thích dòng gốc 501] Cập nhật trọng số cạnh giữa hai goal bằng chiều dài polyline rút gọn.
                self.adj[u][v] = dis

        # [Giải thích dòng gốc 503] Sau khi có cả hai chiều, đồng bộ hoá ma trận để đảm bảo tính đối xứng tốt nhất.
        for u in range(n-1):
            # [Giải thích dòng gốc 504] Chỉ duyệt nửa trên của ma trận vì đồ thị là vô hướng.
            for v in range(u,n):
                # [Giải thích dòng gốc 505] Nếu hai goal chưa có cặp điểm giao hợp lệ thì bỏ qua cặp này.
                if (self.inters[u][v][0][0]==-1): continue

                # [Giải thích dòng gốc 507] Nếu chiều `u -> v` tệ hơn chiều ngược lại thì sao chép từ chiều ngược.
                if (self.adj[u][v]>self.adj[v][u]):
                    # [Giải thích dòng gốc 508] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
                    self.adj[u][v] = self.adj[v][u]
                    # [Giải thích dòng gốc 509] Lấy đường của chiều ngược lại.
                    self.pathTrace[u][v] = self.pathTrace[v][u].copy()
                    # [Giải thích dòng gốc 510] Đảo chiều danh sách điểm để khớp hướng `u -> v`.
                    self.pathTrace[u][v].reverse()
                    # [Giải thích dòng gốc 511] Bỏ qua láng giềng không hợp lệ/đã thăm.
                    continue
                # [Giải thích dòng gốc 512] Ngược lại, nếu chiều `u -> v` tốt hơn thì cập nhật lại chiều `v -> u`.
                if (self.adj[u][v]<self.adj[v][u]):
                    # [Giải thích dòng gốc 513] Đặt trọng số chiều ngược bằng trọng số tốt hơn.
                    self.adj[v][u] = self.adj[u][v]
                    # [Giải thích dòng gốc 514] Sao chép đường đi từ chiều tốt hơn.
                    self.pathTrace[v][u] = self.pathTrace[u][v].copy()
                    # [Giải thích dòng gốc 515] Đảo danh sách điểm để đường phù hợp hướng `v -> u`.
                    self.pathTrace[v][u].reverse()
                    # [Giải thích dòng gốc 516] Bỏ qua láng giềng không hợp lệ/đã thăm.
                    continue


    # [Giải thích dòng gốc 519] Chạy Dijkstra trên đồ thị goal-goal để suy ra khoảng cách ngắn nhất giữa mọi cặp goal qua các cạnh trung gian.
    def dijkstra(self):

        # [Giải thích dòng gốc 521] Khởi tạo ma trận `next`; trong bản này hầu như không dùng về sau.
        self.next = [ [-1 for i in range(self.npos)] for i in range(self.npos)]
        # [Giải thích dòng gốc 522] Ma trận khoảng cách ngắn nhất giữa mọi cặp goal, ban đầu là vô cùng.
        self.dijk = [ [self.mapSize*self.mapSize for i in range(self.npos)] for i in range(self.npos)]
        # [Giải thích dòng gốc 523] Ma trận truy vết cha trên đồ thị goal-goal khi chạy Dijkstra.
        self.dtra = [ [-1 for i in range(self.npos)] for i in range(self.npos)]
        # [Giải thích dòng gốc 524] Chạy Dijkstra riêng từ từng goal làm đỉnh nguồn.
        for root in range(self.npos):
            # [Giải thích dòng gốc 525] Khoảng cách từ một goal đến chính nó bằng 0.
            self.dijk[root][root] = 0
            # [Giải thích dòng gốc 526] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
            pq = queue.PriorityQueue()
            # [Giải thích dòng gốc 527] Đẩy nguồn vào priority queue với chi phí 0.
            pq.put((0,root))

            # [Giải thích dòng gốc 529] Bắt đầu vòng lặp `while`; khối lệnh bên dưới sẽ lặp lại khi điều kiện còn đúng.
            while not pq.empty():
                # [Giải thích dòng gốc 530] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                item = pq.get()
                # [Giải thích dòng gốc 531] Lấy đỉnh goal hiện tại từ phần tử hàng đợi.
                u = item[1]
                # [Giải thích dòng gốc 532] Thử relax cạnh từ `u` sang mọi goal `v` khác.
                for v in range(self.npos):
                    # [Giải thích dòng gốc 533] Nếu đi từ `root` qua `u` tới `v` tốt hơn khoảng cách cũ thì cập nhật.
                    if (self.dijk[root][v] > item[0] + self.adj[u][v]):
                        # [Giải thích dòng gốc 534] Ghi khoảng cách ngắn nhất mới tìm được.
                        self.dijk[root][v] = item[0] + self.adj[u][v]
                        # [Giải thích dòng gốc 535] Lưu đỉnh đứng trước `v` trên đường đi ngắn nhất từ `root`.
                        self.dtra[root][v] = u
                        # [Giải thích dòng gốc 536] Đưa đỉnh `v` vào queue để tiếp tục nới lỏng các cạnh đi ra từ nó.
                        pq.put((self.dijk[root][v],v))


    # [Giải thích dòng gốc 539] Quy trình xây đồ thị bằng Fast Marching cơ bản.
    def buildGraphNormal(self):
        # [Giải thích dòng gốc 540] Bước 1: lan sóng đa nguồn và tạo các cạnh biên cơ bản.
        self.buildForest()
        # [Giải thích dòng gốc 541] Bước 2: dựng đường và trọng số chi tiết giữa từng cặp goal.
        self.twoPointTracing()
        # [Giải thích dòng gốc 542] Bước 3: chạy Dijkstra trên đồ thị goal-goal vừa tạo.
        self.dijkstra()
        # [Giải thích dòng gốc 543] Cập nhật nhãn hiển thị để biết đang dùng biến thể thường.
        self.DFType = "Fast marching"
        

    # [Giải thích dòng gốc 546] Quy trình xây đồ thị bằng Fast Marching Firework nâng cao.
    def buildGraphAdvanced(self):
        # [Giải thích dòng gốc 547] Bước 1: lan sóng đa nguồn và tạo các cạnh biên cơ bản.
        self.buildForest()
        # [Giải thích dòng gốc 548] Bổ sung pha lan nâng cao để tạo nhiều tiếp xúc/cạnh hơn giữa các goal.
        self.buildForestAdvanced()
        # [Giải thích dòng gốc 549] Bước 2: dựng đường và trọng số chi tiết giữa từng cặp goal.
        self.twoPointTracing()
        # [Giải thích dòng gốc 550] Bước 3: chạy Dijkstra trên đồ thị goal-goal vừa tạo.
        self.dijkstra()
        # [Giải thích dòng gốc 551] Cập nhật nhãn hiển thị theo đúng biến thể FMF nâng cao.
        self.DFType = "Fast marching firework"



    # [Giải thích dòng gốc 555] Dựng đường đi toàn cục theo thứ tự goal trong lời giải `sol`.
    def getPath(self,sol):
        # [Giải thích dòng gốc 556] Danh sách kết quả chứa toàn bộ các điểm của quỹ đạo cuối.
        re = []
        # [Giải thích dòng gốc 557] Nối goal đầu tiên vào cuối để khép kín chu trình TSP.
        sol = np.append(sol,sol[0])
        # [Giải thích dòng gốc 558] Lưu goal hiện tại/goal trước đó khi ghép từng chặng.
        pre = int(sol[0])
        # [Giải thích dòng gốc 559] Duyệt lần lượt từng chặng trong chu trình khép kín.
        for i in range(1,self.npos+1):
            # [Giải thích dòng gốc 560] Lấy goal đích của chặng hiện tại.
            cur = int(sol[i])
            # [Giải thích dòng gốc 561] Comment debug để in chỉ số goal đi từ đâu tới đâu.
            #print("From ",pre," to ",cur)
            # [Giải thích dòng gốc 562] Comment debug để in toạ độ thực của hai goal.
            #print("From ",self.deslist[pre]," to ",self.deslist[cur])
            # [Giải thích dòng gốc 563] Bản sao của goal xuất phát chặng hiện tại.
            p1 = int(pre)
            # [Giải thích dòng gốc 564] Bản sao của goal đích chặng hiện tại.
            p2 = int(cur)
            # [Giải thích dòng gốc 565] Danh sách tạm chứa chuỗi đoạn đường ghép giữa `p1` và `p2`.
            t1 = []
            # [Giải thích dòng gốc 566] Lần ngược trên đồ thị goal-goal cho đến khi gặp chính goal xuất phát.
            while(p1!=p2):
                # [Giải thích dòng gốc 567] Lấy đỉnh cha của `p2` trên đường ngắn nhất từ `p1` đến `p2`.
                k = self.dtra[p1][p2]
                # [Giải thích dòng gốc 568] Lấy đường chi tiết của cạnh từ `p2` lùi về cha `k`.
                t2 = list(self.pathTrace[p2][k]).copy() # t2 = p2 -> k
                # [Giải thích dòng gốc 569] Comment debug để in các đỉnh trung gian đang đi qua.
                #print("pass ",k," ... ",p2)
                # [Giải thích dòng gốc 570] Ghép đoạn cạnh hiện tại vào danh sách tạm.
                t1.extend(t2.copy())
                # [Giải thích dòng gốc 571] Dịch điểm đích tạm thời lùi một bước về cha.
                p2 = k

            # [Giải thích dòng gốc 573] Comment gốc nhắc rằng sau vòng lặp, `t1` đang theo chiều ngược về nguồn.
            #t1 = p2 -> p1

            # [Giải thích dòng gốc 575] Đảo lại để đường đi có chiều từ `pre` sang `cur`.
            t1.reverse()
            # [Giải thích dòng gốc 576] Ghép chặng này vào quỹ đạo toàn cục.
            re.extend(t1)
            # [Giải thích dòng gốc 577] Chuẩn bị cho chặng tiếp theo bằng cách dời goal trước thành goal hiện tại.
            pre = cur

            # [Giải thích dòng gốc 579] Comment debug để in toàn bộ điểm của chặng vừa ghép.
            #print(t1,'\n')

        # [Giải thích dòng gốc 581] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re


    # [Giải thích dòng gốc 584] Đọc map từ file text theo định dạng: dòng đầu là kích thước, các dòng sau là ma trận giá trị 0/1/2.
    def get_grid_from_file(self,file_path):

        # [Giải thích dòng gốc 586] Mở file map ở chế độ đọc văn bản.
        f = open(file_path, "r")
        # [Giải thích dòng gốc 587] Đọc toàn bộ nội dung rồi tách thành từng dòng.
        info = f.read().split('\n')

        # [Giải thích dòng gốc 589] Dòng đầu tiên cho biết kích thước map vuông.
        msz = int(info[0])
        # [Giải thích dòng gốc 590] Khởi tạo bộ đếm số goal/checkpoint trong file.
        npos = 0
        # [Giải thích dòng gốc 591] Danh sách để gom toạ độ tất cả goal tìm được.
        points = []

        # [Giải thích dòng gốc 593] Lấy phần còn lại làm các dòng dữ liệu ma trận map.
        gr = info[1:]
        # [Giải thích dòng gốc 594] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(msz):
            # [Giải thích dòng gốc 595] Tách từng dòng thành các token số theo dấu cách.
            gr[i] = gr[i].split(' ')
            # [Giải thích dòng gốc 596] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(msz):
                # [Giải thích dòng gốc 597] Chuyển token chuỗi thành số nguyên.
                gr[i][j] = int(gr[i][j])
                # [Giải thích dòng gốc 598] Nếu ô là goal thì ghi nhận vị trí của nó.
                if (gr[i][j]==2):
                    # [Giải thích dòng gốc 599] Thêm toạ độ goal vào danh sách.
                    points.append((i,j))
                    # [Giải thích dòng gốc 600] Tăng số lượng goal đếm được.
                    npos += 1
        
        # [Giải thích dòng gốc 602] Cập nhật lại kích thước map theo file vừa đọc.
        self.mapSize = msz
        # [Giải thích dòng gốc 603] Lưu số lượng goal thực tế có trong file.
        self.npos = npos
        # [Giải thích dòng gốc 604] Lưu danh sách goal lấy từ file.
        self.deslist = points
        # [Giải thích dòng gốc 605] Lưu toàn bộ ma trận map vào đối tượng.
        self.gridMap = gr
        # [Giải thích dòng gốc 606] Điều chỉnh kích thước marker khi vẽ: map càng lớn thì marker càng nhỏ.
        self.mksz = int(20*20/msz +1)
           


    # [Giải thích dòng gốc 610] Vẽ đường đi cuối cùng trên map bằng matplotlib.
    def drawPath(self,points):
        # [Giải thích dòng gốc 611] Lấy kích thước map cho gọn khi viết công thức vẽ.
        sz = self.mapSize
        # [Giải thích dòng gốc 612] Tạo canvas mới kích thước 8x8 inch với mật độ điểm 80 dpi.
        plt.figure(figsize=(8, 8), dpi=80)
        # [Giải thích dòng gốc 613] Cách đặt trục cũ đã bị comment lại.
        ##plt.axis([ -sz, sz, -sz, sz]) 
        # [Giải thích dòng gốc 614] Đặt giới hạn trục để map hiện đúng hướng khi trục y bị lật dấu.
        plt.axis([ -1, sz, -sz, 1]) 
        # [Giải thích dòng gốc 615] Đặt tiêu đề hình bằng tên thuật toán hiện tại.
        plt.title(self.DFType,fontsize=18)
        # [Giải thích dòng gốc 616] Lấy kích thước marker ra biến cục bộ; phần `mksz = mksz =` là dư thừa nhưng vẫn chạy được.
        mksz = mksz = self.mksz

        # [Giải thích dòng gốc 618] Sao chép danh sách điểm đầu vào để tránh sửa trực tiếp dữ liệu gốc.
        points = list(points).copy()
        # [Giải thích dòng gốc 619] Duyệt từng điểm để đổi hệ toạ độ sang hệ vẽ của matplotlib.
        for i in range(len(points)):
            # [Giải thích dòng gốc 620] Đổi dấu hàng/x để khi vẽ trục y hướng xuống giống ma trận.
            points[i][0] *= -1
        # [Giải thích dòng gốc 621] Tách danh sách điểm thành hai dãy toạ độ để truyền cho matplotlib.
        ys, xs = zip(*points) #create lists of x and y values
        # [Giải thích dòng gốc 622] Vẽ polyline đường đi bằng nét xanh dày.
        plt.plot(xs,ys,color='blue',linewidth = 4) 
        
        # [Giải thích dòng gốc 624] Danh sách toạ độ x của toàn bộ obstacle.
        blx = []
        # [Giải thích dòng gốc 625] Danh sách toạ độ y của toàn bộ obstacle.
        bly = []
        # [Giải thích dòng gốc 626] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 627] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 628] Nếu ô gốc là obstacle thì tăng tổng tích lũy lên 1.
                if (self.gridMap[i][j]==1):
                    # [Giải thích dòng gốc 629] Thêm cột của obstacle vào danh sách x.
                    blx.append(j)
                    # [Giải thích dòng gốc 630] Thêm hàng đã đổi dấu vào danh sách y.
                    bly.append(-i)
        # [Giải thích dòng gốc 631] Vẽ obstacle dưới dạng marker vuông màu đen.
        plt.plot(blx,bly,'ks',markersize = mksz)

        # [Giải thích dòng gốc 633] Danh sách toạ độ x của các goal.
        dx = []
        # [Giải thích dòng gốc 634] Danh sách toạ độ y của các goal.
        dy = []
        # [Giải thích dòng gốc 635] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 636] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 637] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                if (self.gridMap[i][j]==2):
                    # [Giải thích dòng gốc 638] Thêm cột goal vào danh sách x.
                    dx.append(j)
                    # [Giải thích dòng gốc 639] Thêm hàng goal đã đổi dấu vào danh sách y.
                    dy.append(-i)
        # [Giải thích dòng gốc 640] Vẽ các goal bằng marker vuông màu đỏ lớn hơn obstacle.
        plt.plot(dx,dy,'s',color='red',markersize = mksz + 3)

        # [Giải thích dòng gốc 642] Tạo bốn góc của khung ngoài bao bản đồ.
        conner = [[-0.5,0.5], [sz-0.5,0.5], [sz-0.5,-(sz-0.5)], [-0.5,-(sz-0.5)]]
        # [Giải thích dòng gốc 643] Lặp lại góc đầu để khép kín đường viền.
        conner.append(conner[0])
        # [Giải thích dòng gốc 644] Tách toạ độ viền thành hai dãy x-y.
        cnx, cny = zip(*conner) #create lists of x and y values
        # [Giải thích dòng gốc 645] Vẽ khung viền màu đen quanh map.
        plt.plot(cnx,cny,color="black")
        # [Giải thích dòng gốc 646] Ẩn vạch chia trục x để hình gọn hơn.
        plt.xticks([])
        # [Giải thích dòng gốc 647] Ẩn vạch chia trục y.
        plt.yticks([])
        # [Giải thích dòng gốc 648] Hiển thị hình vừa vẽ.
        plt.show()




    # [Giải thích dòng gốc 653] Vẽ các vùng/component của FMF cùng các đường nối giữa các goal.
    def drawFMComponent(self,rmv=[]):
        # [Giải thích dòng gốc 654] Lấy kích thước map cho gọn khi viết công thức vẽ.
        sz = self.mapSize
        # [Giải thích dòng gốc 655] Tạo canvas mới kích thước 8x8 inch với mật độ điểm 80 dpi.
        plt.figure(figsize=(8, 8), dpi=80)
        # [Giải thích dòng gốc 656] Cách đặt trục cũ đã bị comment lại.
        ##plt.axis([ -sz, sz, -sz, sz]) 
        # [Giải thích dòng gốc 657] Đặt giới hạn trục để map hiện đúng hướng khi trục y bị lật dấu.
        plt.axis([ -1, sz, -sz, 1]) 
        # [Giải thích dòng gốc 658] Đặt tiêu đề hình bằng tên thuật toán hiện tại.
        plt.title(self.DFType,fontsize=18)
        # [Giải thích dòng gốc 659] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
        mksz = self.mksz

        # [Giải thích dòng gốc 661] Duyệt danh sách component muốn loại hoặc gộp hiển thị.
        for cmp in rmv:
            # [Giải thích dòng gốc 662] Duyệt từng ô thuộc component đó.
            for pos in self.hold[cmp]:
                # [Giải thích dòng gốc 663] Thay owner lớp 0 bằng owner lớp 1 để phản ánh kết quả pha firework nâng cao.
                self.owner[0][pos[0]][pos[1]] = self.owner[1][pos[0]][pos[1]]

        # [Giải thích dòng gốc 665] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(sz):
            # [Giải thích dòng gốc 666] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(sz):
                # [Giải thích dòng gốc 667] Không tô màu cho obstacle.
                if self.gridMap[i][j] == 1: continue
                # [Giải thích dòng gốc 668] Bỏ qua ô chưa được owner nào chiếm.
                if self.owner[0][i][j] == -1: continue
                # [Giải thích dòng gốc 669] Lấy chỉ số owner để chọn màu.
                col = self.owner[0][i][j]
                # [Giải thích dòng gốc 670] Lấy modulo để không vượt quá danh sách màu đã chuẩn bị.
                col = col % len(self.colorHold)
                # [Giải thích dòng gốc 671] Comment debug cũ liên quan đến việc in đường đang vẽ.
                #print(xs[0],' ',ys[0],' to ',xs[1],' ',ys[1])
                # [Giải thích dòng gốc 672] Tô ô `(i, j)` bằng màu của component chủ quản.
                plt.plot(j,-i,'s',color=self.colorHold[col],markersize = mksz)

        # [Giải thích dòng gốc 674] Danh sách toạ độ x của toàn bộ obstacle.
        blx = []
        # [Giải thích dòng gốc 675] Danh sách toạ độ y của toàn bộ obstacle.
        bly = []
        # [Giải thích dòng gốc 676] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 677] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 678] Nếu ô gốc là obstacle thì tăng tổng tích lũy lên 1.
                if (self.gridMap[i][j]==1):
                    # [Giải thích dòng gốc 679] Thêm cột của obstacle vào danh sách x.
                    blx.append(j)
                    # [Giải thích dòng gốc 680] Thêm hàng đã đổi dấu vào danh sách y.
                    bly.append(-i)
        # [Giải thích dòng gốc 681] Vẽ obstacle dưới dạng marker vuông màu đen.
        plt.plot(blx,bly,'ks',markersize = mksz)

        # [Giải thích dòng gốc 683] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.npos-1):
            # [Giải thích dòng gốc 684] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(i,self.npos):
                # [Giải thích dòng gốc 685] Nếu hai goal không có cạnh/điểm giao hợp lệ thì bỏ qua.
                if (self.inters[i][j][0][0]==-1): continue
                # [Giải thích dòng gốc 686] Lấy đường chi tiết giữa cặp goal để vẽ lên hình.
                points = list(self.pathTrace[i][j]).copy()
                # [Giải thích dòng gốc 687] Comment debug để in cặp goal đang vẽ đường.
                #print("From ",i," to ",j)
                # [Giải thích dòng gốc 688] Duyệt các điểm trên đường để đổi dấu tung độ trước khi vẽ.
                for k in range(len(points)):
                    # [Giải thích dòng gốc 689] Chỉ đổi dấu các tung độ dương để tránh lật lại các điểm đã âm sẵn.
                    if (points[k][0] > 0):
                        # [Giải thích dòng gốc 690] Đổi dấu tung độ cho khớp hệ trục hiển thị.
                        points[k][0] *= -1
                # [Giải thích dòng gốc 691] Comment debug để in toàn bộ danh sách điểm sau khi đổi hệ trục.
                #print(points,"\n")
                # [Giải thích dòng gốc 692] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                ys, xs = zip(*points) #create lists of x and y values
                # [Giải thích dòng gốc 693] Vẽ đường nối giữa hai goal bằng nét màu đỏ thẫm.
                plt.plot(xs,ys,color='crimson',linewidth = 3) 
    

        # [Giải thích dòng gốc 696] Danh sách toạ độ x của các goal.
        dx = []
        # [Giải thích dòng gốc 697] Danh sách toạ độ y của các goal.
        dy = []
        # [Giải thích dòng gốc 698] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 699] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 700] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                if (self.gridMap[i][j]==2):
                    # [Giải thích dòng gốc 701] Thêm cột goal vào danh sách x.
                    dx.append(j)
                    # [Giải thích dòng gốc 702] Thêm hàng goal đã đổi dấu vào danh sách y.
                    dy.append(-i)
        # [Giải thích dòng gốc 703] Vẽ goal chồng lên trên cùng với kích thước nổi bật hơn.
        plt.plot(dx,dy,'s',color='red',markersize = mksz+4)

        # [Giải thích dòng gốc 705] Tạo bốn góc của khung ngoài bao bản đồ.
        conner = [[-0.5,0.5], [sz-0.5,0.5], [sz-0.5,-(sz-0.5)], [-0.5,-(sz-0.5)]]
        # [Giải thích dòng gốc 706] Lặp lại góc đầu để khép kín đường viền.
        conner.append(conner[0])
        # [Giải thích dòng gốc 707] Tách toạ độ viền thành hai dãy x-y.
        cnx, cny = zip(*conner) #create lists of x and y values
        # [Giải thích dòng gốc 708] Vẽ khung viền màu đen quanh map.
        plt.plot(cnx,cny,color="black")
        # [Giải thích dòng gốc 709] Ẩn vạch chia trục x để hình gọn hơn.
        plt.xticks([])
        # [Giải thích dòng gốc 710] Ẩn vạch chia trục y.
        plt.yticks([])
        # [Giải thích dòng gốc 711] Hiển thị hình vừa vẽ.
        plt.show()


    # [Giải thích dòng gốc 714] Vẽ trường sóng/khoảng cách `dista` sau khi lan, tô màu theo giá trị khoảng cách.
    def drawDijkstraWave(self,rmv=[]):
        # [Giải thích dòng gốc 715] Lấy kích thước map cho gọn khi viết công thức vẽ.
        sz = self.mapSize
        # [Giải thích dòng gốc 716] Tạo canvas mới kích thước 8x8 inch với mật độ điểm 80 dpi.
        plt.figure(figsize=(8, 8), dpi=80)
        # [Giải thích dòng gốc 717] Cách đặt trục cũ đã bị comment lại.
        ##plt.axis([ -sz, sz, -sz, sz]) 
        # [Giải thích dòng gốc 718] Đặt giới hạn trục để map hiện đúng hướng khi trục y bị lật dấu.
        plt.axis([ -1, sz, -sz, 1]) 
        # [Giải thích dòng gốc 719] Đặt tiêu đề hình bằng tên thuật toán hiện tại.
        plt.title(self.DFType,fontsize=18)
        # [Giải thích dòng gốc 720] Gán hoặc lấy một thuộc tính của đối tượng để dùng trong ngữ cảnh hiện tại.
        mksz = self.mksz
        
        # [Giải thích dòng gốc 722] Danh sách toạ độ x của toàn bộ obstacle.
        blx = []
        # [Giải thích dòng gốc 723] Danh sách toạ độ y của toàn bộ obstacle.
        bly = []
        # [Giải thích dòng gốc 724] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 725] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 726] Nếu ô gốc là obstacle thì tăng tổng tích lũy lên 1.
                if (self.gridMap[i][j]==1):
                    # [Giải thích dòng gốc 727] Thêm cột của obstacle vào danh sách x.
                    blx.append(j)
                    # [Giải thích dòng gốc 728] Thêm hàng đã đổi dấu vào danh sách y.
                    bly.append(-i)
        # [Giải thích dòng gốc 729] Vẽ obstacle dưới dạng marker vuông màu đen.
        plt.plot(blx,bly,'ks',markersize = mksz)

        # [Giải thích dòng gốc 731] Nhánh này luôn sai nên thực chất bị vô hiệu; có lẽ là code thử nghiệm hiển thị màu theo tay.
        if (0==1):
            # [Giải thích dòng gốc 732] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for i in range(self.mapSize):
                # [Giải thích dòng gốc 733] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
                for j in range(self.mapSize):
                    # [Giải thích dòng gốc 734] Nếu là ô trống đã được một owner chiếm thì chuẩn bị tô màu theo khoảng cách.
                    if (self.gridMap[i][j]==0 and self.owner[0][i][j]!=-1):
                        # [Giải thích dòng gốc 735] Comment debug cũ để kiểm tra các ô thuộc wavefront.
                        #print("wave")
                        # [Giải thích dòng gốc 736] Chuẩn hoá khoảng cách về một giá trị dùng để tạo màu xám-xanh thủ công.
                        val = (self.dista[i][j]*2+self.mapSize/20)/self.mapSize
                        # [Giải thích dòng gốc 737] Đưa giá trị chuẩn hoá sang miền 0-255.
                        val = int(val*255)
                        # [Giải thích dòng gốc 738] Chặn trên để đảm bảo mã màu hợp lệ.
                        if (val>255): val = 255
                        # [Giải thích dòng gốc 739] Đổi sang chuỗi hex.
                        he = hex(val)
                        # [Giải thích dòng gốc 740] Bỏ tiền tố `0x` trong chuỗi hex.
                        he = he[2:]
                        # [Giải thích dòng gốc 741] Nếu chỉ có 1 ký tự thì thêm số 0 ở trước để đủ 2 chữ số.
                        if (len(he)==1): he = '0' + he
                        # [Giải thích dòng gốc 742] Ghép chuỗi thành mã màu RGB dạng `#RRGGBB` với B cố định là `DF`.
                        col = '#' + he + he + 'DF' 
                        # [Giải thích dòng gốc 743] Vẽ ô với màu vừa tự tính thủ công.
                        plt.plot(j,-i,'s',color=col,markersize = mksz)
        # [Giải thích dòng gốc 744] Nhánh được dùng thật sự: tô màu theo colormap của matplotlib.
        else:
            # [Giải thích dòng gốc 745] Khối comment cũ cho phương án tô màu rời rạc theo `colorWave`.
            #for i in range(self.mapSize):
            # [Giải thích dòng gốc 746] Dòng comment tiếp nối của đoạn thử nghiệm cũ.
            #    for j in range(self.mapSize):
            # [Giải thích dòng gốc 747] Điều kiện của phương án tô màu cũ.
            #        if (self.gridMap[i][j]==0 and self.owner[0][i][j]!=-1):
            # [Giải thích dòng gốc 748] Tính chỉ số màu theo khoảng cách trong phương án cũ.
            #            val = int(self.dista[i][j]*2)
            # [Giải thích dòng gốc 749] Lấy modulo để ánh xạ vào bảng màu rời rạc.
            #            val = val % len(self.colorWave)
            # [Giải thích dòng gốc 750] Vẽ ô theo bảng màu rời rạc trong phương án cũ.
            #            plt.plot(j,-i,'s',color=self.colorWave[val],markersize = mksz)
            # [Giải thích dòng gốc 751] Danh sách toạ độ x của các ô sẽ được tô theo wave.
            x2 = []
            # [Giải thích dòng gốc 752] Danh sách toạ độ y của các ô sẽ được tô theo wave.
            y2 = []
            # [Giải thích dòng gốc 753] Danh sách giá trị màu, ở đây lấy từ âm của `dista`.
            z2 = []
            # [Giải thích dòng gốc 754] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for i in range(self.mapSize):
                # [Giải thích dòng gốc 755] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
                for j in range(self.mapSize):
                    # [Giải thích dòng gốc 756] Nếu là ô trống đã được một owner chiếm thì chuẩn bị tô màu theo khoảng cách.
                    if (self.gridMap[i][j]==0 and self.owner[0][i][j]!=-1):
                        # [Giải thích dòng gốc 757] Nếu owner của ô nằm trong danh sách loại bỏ thì không vẽ ô đó.
                        if self.owner[0][i][j] in rmv: continue
                        # [Giải thích dòng gốc 758] Thêm cột ô vào tập toạ độ x.
                        x2.append(j)
                        # [Giải thích dòng gốc 759] Thêm hàng đã đổi dấu vào tập toạ độ y.
                        y2.append(-i)
                        # [Giải thích dòng gốc 760] Thêm giá trị màu; dấu âm làm vùng gần nguồn có màu nổi bật theo colormap.
                        z2.append(-self.dista[i][j])
            # [Giải thích dòng gốc 761] Comment debug cũ để kiểm tra số lượng điểm x.
            #print(len(x2))
            # [Giải thích dòng gốc 762] Comment debug cũ để kiểm tra số lượng điểm y.
            #print(len(y2))
            # [Giải thích dòng gốc 763] Comment debug cũ để kiểm tra số lượng giá trị màu.
            #print(len(z2))
            # [Giải thích dòng gốc 764] Vẽ toàn bộ wavefront bằng scatter, dùng colormap `jet` và marker vuông.
            plt.scatter(x2, y2, c=z2, cmap="jet",marker='s',s = mksz*mksz)

        # [Giải thích dòng gốc 766] Danh sách toạ độ x của các goal.
        dx = []
        # [Giải thích dòng gốc 767] Danh sách toạ độ y của các goal.
        dy = []
        # [Giải thích dòng gốc 768] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
        for i in range(self.mapSize):
            # [Giải thích dòng gốc 769] Bắt đầu một vòng lặp để duyệt qua tập phần tử/phạm vi chỉ số tương ứng.
            for j in range(self.mapSize):
                # [Giải thích dòng gốc 770] Đặt một điều kiện rẽ nhánh; nếu điều kiện đúng thì thực hiện khối bên dưới.
                if (self.gridMap[i][j]==2):
                    # [Giải thích dòng gốc 771] Thêm cột goal vào danh sách x.
                    dx.append(j)
                    # [Giải thích dòng gốc 772] Thêm hàng goal đã đổi dấu vào danh sách y.
                    dy.append(-i)
        # [Giải thích dòng gốc 773] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
        plt.plot(dx,dy,'s',color='red',markersize = mksz)

        # [Giải thích dòng gốc 775] Tạo bốn góc của khung ngoài bao bản đồ.
        conner = [[-0.5,0.5], [sz-0.5,0.5], [sz-0.5,-(sz-0.5)], [-0.5,-(sz-0.5)]]
        # [Giải thích dòng gốc 776] Lặp lại góc đầu để khép kín đường viền.
        conner.append(conner[0])
        # [Giải thích dòng gốc 777] Tách toạ độ viền thành hai dãy x-y.
        cnx, cny = zip(*conner) #create lists of x and y values
        # [Giải thích dòng gốc 778] Vẽ khung viền màu đen quanh map.
        plt.plot(cnx,cny,color="black")
        # [Giải thích dòng gốc 779] Ẩn vạch chia trục x để hình gọn hơn.
        plt.xticks([])
        # [Giải thích dòng gốc 780] Ẩn vạch chia trục y.
        plt.yticks([])
        # [Giải thích dòng gốc 781] Hiển thị hình vừa vẽ.
        plt.show()





























    # [Giải thích dòng gốc 811] Ép một toạ độ bất kỳ về trong biên hợp lệ của map.
    def inBound(self,pos):
        # [Giải thích dòng gốc 812] Làm tròn toạ độ về số nguyên gần nhất trước khi kẹp biên.
        pos = np.round(pos,0)
        # [Giải thích dòng gốc 813] Nếu hàng âm thì kéo về biên trên.
        if (pos[0]<0):
            # [Giải thích dòng gốc 814] Kẹp giá trị hàng nhỏ nhất là 0.
            pos[0] = 0
        # [Giải thích dòng gốc 815] Nếu hàng vượt biên dưới thì kéo về hàng cuối.
        if (pos[0]>self.mapSize-1):
            # [Giải thích dòng gốc 816] Kẹp hàng lớn nhất bằng `mapSize - 1`.
            pos[0] = self.mapSize-1

        # [Giải thích dòng gốc 818] Nếu cột âm thì kéo về biên trái.
        if (pos[1]<0):
            # [Giải thích dòng gốc 819] Kẹp cột nhỏ nhất là 0.
            pos[1] = 0
        # [Giải thích dòng gốc 820] Nếu cột vượt biên phải thì kéo về cột cuối.
        if (pos[1]>self.mapSize-1):
            # [Giải thích dòng gốc 821] Kẹp cột lớn nhất bằng `mapSize - 1`.
            pos[1] = self.mapSize-1

        # [Giải thích dòng gốc 823] Chuyển kết quả thành mảng NumPy kiểu số nguyên.
        pos = np.array((int(pos[0]),int(pos[1])))
        # [Giải thích dòng gốc 824] Trả kết quả hiện tại ra khỏi hàm.
        return pos



    # [Giải thích dòng gốc 828] Tìm ô trống gần nhất bắt đầu từ vị trí `pos` bằng BFS trên 8 hướng.
    def nearestFree(self,pos):
        # [Giải thích dòng gốc 829] Đảm bảo điểm xuất phát ban đầu nằm trong map.
        pos = self.inBound(pos)
        # [Giải thích dòng gốc 830] Ma trận đánh dấu đã thăm/tạm dùng cho BFS.
        d = np.zeros((self.mapSize,self.mapSize))
        # [Giải thích dòng gốc 831] Bảng dịch hàng cho 8 hướng BFS.
        hx = [0,1,0,-1,1,1,-1,-1]
        # [Giải thích dòng gốc 832] Bảng dịch cột cho 8 hướng BFS.
        hy = [-1,0,1,0,-1,1,-1,1]


        # [Giải thích dòng gốc 835] Con trỏ đầu hàng đợi tự cài đặt bằng mảng NumPy.
        head = 0
        # [Giải thích dòng gốc 836] Con trỏ cuối hàng đợi.
        tail = 0
        # [Giải thích dòng gốc 837] Khởi tạo hàng đợi với duy nhất điểm xuất phát.
        Q = np.array([[pos[0],pos[1]]])
        # [Giải thích dòng gốc 838] Tiếp tục BFS khi vẫn còn phần tử chưa xử lý trong hàng đợi.
        while (head<=tail):
            
            # [Giải thích dòng gốc 840] Lấy phần tử đầu hàng đợi ra xét.
            cur = Q[head]
            # [Giải thích dòng gốc 841] Tăng con trỏ đầu để pop logic phần tử vừa xử lý.
            head+=1

            # [Giải thích dòng gốc 843] Nếu ô hiện tại không phải obstacle thì đây là ô trống gần nhất cần tìm.
            if (self.gridMap[int(cur[0])][int(cur[1])] != 1):
                # [Giải thích dòng gốc 844] Trả ngay toạ độ hợp lệ của ô trống gần nhất.
                return self.inBound(cur)

            # [Giải thích dòng gốc 846] Tạo danh sách chỉ số 8 hướng để duyệt láng giềng.
            rag = np.array((0,1,2,3,4,5,6,7))

            # [Giải thích dòng gốc 848] Duyệt lần lượt 8 hướng lân cận.
            for i in rag:
                # [Giải thích dòng gốc 849] Tính hàng của láng giềng mới.
                x = cur[0] + hx[int(i)]
                # [Giải thích dòng gốc 850] Tính cột của láng giềng mới.
                y = cur[1] + hy[int(i)]
                # [Giải thích dòng gốc 851] Nếu ra ngoài map hoặc đã xét rồi thì bỏ qua.
                if (x<0) or (x>self.mapSize-1) or (y<0) or (y>self.mapSize-1) or (d[int(x)][int(y)] != 0):
                    # [Giải thích dòng gốc 852] Bỏ qua láng giềng không hợp lệ/đã thăm.
                    continue
                    
                # [Giải thích dòng gốc 854] Dòng này có vẻ là lỗi hoặc dư thừa, vì nó đặt lại 0 thay vì đánh dấu đã thăm bằng 1.
                d[int(x)][int(y)] = 0
                # [Giải thích dòng gốc 855] Thêm láng giềng mới vào cuối hàng đợi BFS.
                Q = np.append(Q,np.array([[x,y]]),0)
                # [Giải thích dòng gốc 856] Tăng con trỏ cuối tương ứng với phần tử mới được thêm.
                tail+=1
        

    # [Giải thích dòng gốc 859] Tính khoảng cách hoặc truy vết đường đi từ `start` đến `end` bằng một BFS/label-propagation kiểu cũ.
    def getDis(self, start, end, returnPath = 0):

        # [Giải thích dòng gốc 861] Đưa điểm bắt đầu về trong biên map.
        start = self.inBound(start)
        # [Giải thích dòng gốc 862] Đưa điểm kết thúc về trong biên map.
        end = self.inBound(end)

        # [Giải thích dòng gốc 864] Nếu chưa từng tính toàn bộ trường khoảng cách xuất phát từ `start` thì mới chạy BFS.
        if self.checked[start[0]][start[1]] == 0:
            
            # [Giải thích dòng gốc 866] Đánh dấu đã cache khoảng cách từ start này.
            self.checked[start[0]][start[1]] = 1
            # [Giải thích dòng gốc 867] Ma trận khoảng cách tạm cho BFS hiện tại.
            d = np.zeros((self.mapSize,self.mapSize))
            # [Giải thích dòng gốc 868] Ma trận cha để lần ngược đường đi nếu cần.
            trace = np.zeros((self.mapSize,self.mapSize,2))

            # [Giải thích dòng gốc 870] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
            hx = [0,1,0,-1,1,1,-1,-1]
            # [Giải thích dòng gốc 871] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
            hy = [-1,0,1,0,-1,1,-1,1]


            # [Giải thích dòng gốc 874] Duyệt toàn map để tiền xử lý các obstacle.
            for i in range(0,self.mapSize):
                # [Giải thích dòng gốc 875] Duyệt từng cột tương ứng.
                for j in range(0,self.mapSize):

                    # [Giải thích dòng gốc 877] Nếu ô là vật cản thì gán nhãn đặc biệt.
                    if self.gridMap[i][j] == 1:
                        # [Giải thích dòng gốc 878] Dùng `-1` để đánh dấu obstacle trong ma trận khoảng cách.
                        d[i][j] = -1

            # [Giải thích dòng gốc 880] Đặt khoảng cách khởi tạo của start là 1 thay vì 0; vì thế lúc trả kết quả sẽ trừ lại 1.
            d[int(start[0])][int(start[1])] = 1
            # [Giải thích dòng gốc 881] Khởi tạo biến bằng 0.
            head = 0
            # [Giải thích dòng gốc 882] Khởi tạo biến bằng 0.
            tail = 0
            # [Giải thích dòng gốc 883] Khởi tạo hàng đợi BFS bằng điểm xuất phát.
            Q = np.array([[start[0],start[1]]])
            # [Giải thích dòng gốc 884] Bắt đầu vòng lặp `while`; khối lệnh bên dưới sẽ lặp lại khi điều kiện còn đúng.
            while (head<=tail):
                
                # [Giải thích dòng gốc 886] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                cur = Q[head]
                # [Giải thích dòng gốc 887] Thực hiện phép gán/cập nhật giá trị cho biến hiện tại.
                head+=1

                # [Giải thích dòng gốc 889] Comment gốc: đoạn sau xét các bước đi song song 4 hướng.
                #di song song
                # [Giải thích dòng gốc 890] Duyệt 4 hướng thẳng trước.
                for i in range(4):
                    # [Giải thích dòng gốc 891] Tạo biến chứa toạ độ láng giềng đang xét.
                    cur2 = np.zeros(2)
                    # [Giải thích dòng gốc 892] Tính hàng láng giềng theo hướng `i`.
                    cur2[0] = cur[0] + hx[i]
                    # [Giải thích dòng gốc 893] Tính cột láng giềng theo hướng `i`.
                    cur2[1] = cur[1] + hy[i]

                    # [Giải thích dòng gốc 895] Kiểm tra tính hợp lệ của láng giềng. Lưu ý: trong file hiện tại không thấy định nghĩa `validPos`, đây có vẻ là tên cũ của `validpos`.
                    if self.validPos(cur2):
                        # [Giải thích dòng gốc 896] Nếu ô chưa được gán khoảng cách thì cập nhật và đưa vào queue.
                        if d[int(cur2[0])][int(cur2[1])]==0:
                            # [Giải thích dòng gốc 897] Bước đi thẳng có chi phí 1.
                            d[int(cur2[0])][int(cur2[1])] = d[int(cur[0])][int(cur[1])] + 1
                            # [Giải thích dòng gốc 898] Lưu cha truy vết của ô mới là ô hiện tại.
                            trace[int(cur2[0])][int(cur2[1])] = cur
                            # [Giải thích dòng gốc 899] Đưa ô mới vào cuối hàng đợi.
                            Q = np.append(Q,np.array([[cur2[0],cur2[1]]]),0)
                            # [Giải thích dòng gốc 900] Tăng con trỏ cuối của hàng đợi.
                            tail+=1
                # [Giải thích dòng gốc 901] Comment gốc: đoạn sau xét các bước đi chéo.
                #di cheo
                # [Giải thích dòng gốc 902] Duyệt 4 hướng chéo.
                for i in range(4,8):
                    # [Giải thích dòng gốc 903] Tạo biến chứa toạ độ láng giềng đang xét.
                    cur2 = np.zeros(2)
                    # [Giải thích dòng gốc 904] Tính hàng láng giềng theo hướng `i`.
                    cur2[0] = cur[0] + hx[i]
                    # [Giải thích dòng gốc 905] Tính cột láng giềng theo hướng `i`.
                    cur2[1] = cur[1] + hy[i]
                    # [Giải thích dòng gốc 906] Ô phụ thứ nhất để kiểm tra cắt góc khi đi chéo.
                    cur3 = np.zeros(2)
                    # [Giải thích dòng gốc 907] Giữ nguyên hàng của ô hiện tại cho ô phụ thứ nhất.
                    cur3[0] = cur[0]
                    # [Giải thích dòng gốc 908] Dùng cột của đích chéo cho ô phụ thứ nhất.
                    cur3[1] = cur2[1]
                    # [Giải thích dòng gốc 909] Ô phụ thứ hai để kiểm tra cắt góc.
                    cur4 = np.zeros(2)
                    # [Giải thích dòng gốc 910] Dùng hàng của đích chéo cho ô phụ thứ hai.
                    cur4[0] = cur2[0]
                    # [Giải thích dòng gốc 911] Giữ nguyên cột của ô hiện tại cho ô phụ thứ hai.
                    cur4[1] = cur[1]

                    # [Giải thích dòng gốc 913] Chỉ cho đi chéo nếu cả ô đích và hai ô kề góc đều hợp lệ.
                    if (self.validPos(cur2) and self.validPos(cur3) and self.validPos(cur4)):
                        # [Giải thích dòng gốc 914] Nếu ô chưa được gán khoảng cách thì cập nhật và đưa vào queue.
                        if d[int(cur2[0])][int(cur2[1])]==0:
                            # [Giải thích dòng gốc 915] Bước đi chéo có chi phí căn 2.
                            d[int(cur2[0])][int(cur2[1])] = d[int(cur[0])][int(cur[1])] + math.sqrt(2)
                            # [Giải thích dòng gốc 916] Lưu cha truy vết của ô mới là ô hiện tại.
                            trace[int(cur2[0])][int(cur2[1])] = cur
                            # [Giải thích dòng gốc 917] Đưa ô mới vào cuối hàng đợi.
                            Q = np.append(Q,np.array([[cur2[0],cur2[1]]]),0)
                            # [Giải thích dòng gốc 918] Tăng con trỏ cuối của hàng đợi.
                            tail+=1

            # [Giải thích dòng gốc 920] Cache ma trận khoảng cách xuất phát từ `start`. Lưu ý: `Gbfs` không thấy khởi tạo trong file này nên có thể là phần legacy chưa hoàn chỉnh.
            self.Gbfs[start[0]][start[1]] = np.copy(d)
            # [Giải thích dòng gốc 921] Cache ma trận trace tương ứng cho `start`; `Gtrace` cũng có vẻ là thuộc tính legacy.
            self.Gtrace[start[0]][start[1]] = np.copy(trace)

        # [Giải thích dòng gốc 923] Lấy lại ma trận khoảng cách đã cache của điểm start.
        d = self.Gbfs[start[0]][start[1]]
        # [Giải thích dòng gốc 924] Lấy lại ma trận cha truy vết đã cache của điểm start.
        trace = self.Gtrace[start[0]][start[1]]

        # [Giải thích dòng gốc 926] Nếu người gọi chỉ muốn khoảng cách, không cần đường đi, thì trả luôn số đo.
        if returnPath == 0:
            # [Giải thích dòng gốc 927] Trả khoảng cách tới `end`; trừ 1 để bù cho việc khởi tạo start bằng 1.
            return d[int(end[0])][int(end[1])] - 1

        # [Giải thích dòng gốc 929] Nếu cần đường đi thì bắt đầu lần ngược từ đích.
        cur = np.copy(end)
        # [Giải thích dòng gốc 930] Khởi tạo đường đi với điểm đầu tiên là chính đích.
        re = np.array([[cur[0],cur[1]]])
        # [Giải thích dòng gốc 931] Tiếp tục lần ngược cho đến khi quay về đúng điểm start.
        while ((cur==start).all() == False):
            # [Giải thích dòng gốc 932] Nhảy từ ô hiện tại về cha của nó.
            cur = np.copy(trace[int(cur[0])][int(cur[1])])
            # [Giải thích dòng gốc 933] Ghép cha vừa tìm được vào cuối danh sách đường đi.
            re = np.append(re,np.array([[cur[0],cur[1]]]),0)

        # [Giải thích dòng gốc 935] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re


    # [Giải thích dòng gốc 938] Ghép đường đầy đủ đi qua lần lượt danh sách điểm `posList` rồi quay về điểm đầu.
    def getFullPath(self,posList,posSize):
        # [Giải thích dòng gốc 939] Nối lại điểm đầu vào cuối danh sách để khép vòng.
        posList = np.append( posList,np.array([[posList[0][0],posList[0][1]]]),0)
        # [Giải thích dòng gốc 940] Tăng số lượng điểm vì vừa thêm một điểm cuối.
        posSize += 1
        # [Giải thích dòng gốc 941] Khởi tạo đường đầy đủ bằng điểm đầu tiên.
        trace = np.array([[posList[0][0],posList[0][1]]])

        # [Giải thích dòng gốc 943] Duyệt từng cặp điểm liên tiếp trong chu trình.
        for i in range(1,posSize):

            # [Giải thích dòng gốc 945] Lấy đường đi chi tiết giữa hai điểm liên tiếp bằng hàm `getDis`.
            path = self.getDis(posList[i],posList[i-1],1)
            # [Giải thích dòng gốc 946] Duyệt mọi điểm trên đoạn đường con vừa truy vết được.
            for pos in path:
                # [Giải thích dòng gốc 947] Nối từng điểm vào đường đi tổng thể.
                trace = np.append(trace,np.array([[int(pos[0]),int(pos[1])]]),0)

        # [Giải thích dòng gốc 949] Trả kết quả hiện tại ra khỏi hàm.
        return trace


    # [Giải thích dòng gốc 952] Sinh một thứ tự/đường đi ngẫu nhiên qua các goal; phần đầu hàm là code cũ, cuối hàm mới là thứ được dùng thật.
    def getRandomPath(self):
        # [Giải thích dòng gốc 953] Lấy số goal hiện có.
        pos = self.npos
        # [Giải thích dòng gốc 954] Khởi tạo mảng tạm để chứa một tập điểm ngẫu nhiên.
        re = np.zeros((pos,2))
        # [Giải thích dòng gốc 955] Giữ điểm đầu tiên của danh sách goal ở vị trí đầu.
        re[0] = self.deslist[0]
        # [Giải thích dòng gốc 956] Giữ goal cuối ở vị trí cuối trong phương án cũ.
        re[pos-1] = self.deslist[pos-1]
        # [Giải thích dòng gốc 957] Sinh ngẫu nhiên các điểm ở giữa trong phương án cũ.
        for i in range (1,pos-1):
            # [Giải thích dòng gốc 958] Tạo biến tạm cho toạ độ ngẫu nhiên mới.
            newPos = np.zeros(2)
            # [Giải thích dòng gốc 959] Sinh ngẫu nhiên hàng trong map.
            newPos[0] = np.random.randint(0, self.mapSize)
            # [Giải thích dòng gốc 960] Sinh ngẫu nhiên cột trong map.
            newPos[1] = np.random.randint(0, self.mapSize)

            # [Giải thích dòng gốc 962] Nếu rơi vào obstacle thì dời sang ô trống gần nhất.
            newPos = self.nearestFree(newPos)
            # [Giải thích dòng gốc 963] Gán điểm ngẫu nhiên hợp lệ vào danh sách tạm.
            re[i] = newPos

        # [Giải thích dòng gốc 965] Ngay sau đó tác giả bỏ toàn bộ phương án cũ và reset `re` thành danh sách các goal thật.
        re = np.copy(self.deslist)
        # [Giải thích dòng gốc 966] Xáo ngẫu nhiên thứ tự các goal để tạo một lời giải ban đầu ngẫu nhiên.
        np.random.shuffle(re)
        # [Giải thích dòng gốc 967] Trả về thứ tự goal đã xáo ngẫu nhiên.
        return re
        


