import random


def fix_map(filename):
    with open(filename, 'r') as f:
        lst = []
        for line in f:
            row = list(map(int, line.split()))
            lst.append(row)

        # Thiếu chiều rộng
        n = 200 - len(lst[0])
        if n > 0:
            a = 0
            b = 0
            a = n//2
            if n % 2 == 0:
                b = a
            else:
                b = a + 1
            front_lst = []
            end_lst = []
            for i in range(a):
                front_lst.append(1)
            for i in range(b):
                end_lst.append(1)
            for j in range(200):
                lst[j] = front_lst + lst[j] + end_lst

        # Thiếu chiều dài
        m = 200 - len(lst)
        if m > 0:
            a = 0
            b = 0
            a = m//2
            if m % 2 == 0:
                b = a
            else:
                b = a + 1
            front_lst = []
            end_lst = []
            lst_tmp = []
            for i in range(200):
                lst_tmp.append(1)
            for i in range(a):
                front_lst.append(lst_tmp)
            for i in range(b):
                end_lst.append(lst_tmp)

            lst = front_lst + lst + end_lst

    with open(filename, 'w') as f:
        for row in lst:
            line = ' '.join(map(str, row))
            f.write(line + '\n')
    f.close()


def check_neighbor(i, j, lst):
    n = len(lst)
    if lst[i][j] == 0:
        if lst[i - 1][j] + lst[i + 1][j] + lst[i][j - 1] + lst[i][j + 1] + lst[i - 1][j - 1] + lst[i - 1][j + 1] + lst[i + 1][j - 1] + lst[i + 1][j + 1] == 0:
            return True
    return False


def thay_doi_ma_tran(matrix, m):
    n = len(matrix)
    lst = [row.copy() for row in matrix]  # Tạo bản sao của ma trận để không ảnh hưởng đến ma trận gốc

    zeros = []  # Danh các điểm chứa bit 0
    for i in range(1, n - 1):
        for j in range(1, n - 1):
            if check_neighbor(i, j, lst):
                zeros.append((i, j))

    random.shuffle(zeros)  # Xáo trộn danh sách các điểm 0
    x_0, y_0 = zeros[0]
    lst[x_0][y_0] = 3
    count = 1
    while count < m:
        x, y = zeros[count]
        lst[x][y] = 2  # Thay thế điểm 0 bằng điểm 2
        count += 1

    return lst


def read_map_from_file(filename):
    with open(filename, 'r') as f:
        lst = []
        for line in f:
            row = list(map(int, line.split()))
            for i in range(len(row)):
                if row[i] > 1:
                    row[i] = 0
            lst.append(row)
    f.close()
    return lst


def write_map_to_file(filename, lst):
    with open(filename, 'w') as f:
        for row in lst:
            line = ' '.join(map(str, row))
            f.write(line + '\n')
    f.close()


def gen_random_map(filename, n_point):
    matrix = read_map_from_file(filename)
    updated_matrix = thay_doi_ma_tran(matrix, n_point)
    write_map_to_file(filename, updated_matrix)

