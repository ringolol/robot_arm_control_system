#ifndef RA_MATRIX_H
#define RA_MATRIX_H

class RaMatrix {
public:
    short col_n, row_n;
    float** data;

    RaMatrix(const RaMatrix& other) : RaMatrix(other.row_n, other.col_n) {
        copy(other);
    }

    RaMatrix(float* A, short r_n, short c_n) : RaMatrix(r_n, c_n) {
        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                data[r][c] = A[r * col_n + c];
            }
        }
    }

    RaMatrix(short r_n=0, short c_n=0) {
        init(r_n, c_n);
    }

    ~RaMatrix() {
        del();
    }

    RaMatrix operator*(RaMatrix other) {
        RaMatrix res(row_n, other.col_n);

        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < other.col_n; c++) {
                res.data[r][c] = 0;
                for (short k = 0; k < col_n; k++)
                {
                    res.data[r][c] += data[r][k] * other.data[k][c];
                }
            }
        }
        return res;
    }

    RaMatrix operator+(RaMatrix other) {
        RaMatrix res(row_n, col_n);

        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                res.data[r][c] = data[r][c] + other.data[r][c];
            }
        }
        return res;
    }

    RaMatrix operator-(RaMatrix other) {
        RaMatrix res(row_n, col_n);

        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                res.data[r][c] = data[r][c] - other.data[r][c];
            }
        }
        return res;
    }

    RaMatrix operator*(float cnst) {
        RaMatrix res(row_n, col_n);

        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                res.data[r][c] = data[r][c] * cnst;
            }
        }
        return res;
    }

    RaMatrix& operator=(RaMatrix other) {
        if (row_n != other.row_n || col_n != other.col_n) {
            del();
            init(other.row_n, other.col_n);
            copy(other);
        }
        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                data[r][c] = other.data[r][c];
            }
        }
        return *this;
    }

    RaMatrix tr() {
        RaMatrix res(row_n, col_n);
        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                res.data[r][c] = data[c][r];
            }
        }
        return res;
    }

    RaMatrix inv_diag() {
        RaMatrix res(row_n, col_n);
        for (short i = 0; i < row_n; i++) {
            res.data[i][i] = 1.0f / data[i][i];
        }
        return res;
    }

     void print() {
        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                Serial.print(data[r][c]);
                Serial.print('\t');
            }
            Serial.print('\n');
        }
    }

private:
    void init(short r_n=0, short c_n=0) {
        row_n = r_n;
        col_n = c_n;

        data = new float*[row_n];
        for (short r = 0; r < row_n; r++) {
            data[r] = new float[col_n];
        }
    }

    void copy(const RaMatrix &other) {
        for (short r = 0; r < row_n; r++) {
            for (short c = 0; c < col_n; c++) {
                data[r][c] = other.data[r][c];
            }
        }
    }

    void del() {
        for (short r = 0; r < row_n; r++) {
            delete[] data[r];
        }
        delete[] data;
    }
};

#endif