
#include <matrix.h>
#include <stdlib.h>
#include <math.h>

#ifdef MAT_EN_PRINT
#include <stdio.h>
#endif


////////////////////////////////////////////////////////////////////////////////
/// Macos
////////////////////////////////////////////////////////////////////////////////

#define MAT_IDX(m, row, col)                ((m)->cols * (row) + (col))


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

int matrix_init(matrix *m, size_t rows, size_t cols){
    if(rows == 0 || cols == 0)
        return MAT_ERR_SIZE;
    m->data = calloc(rows * cols, sizeof(float));
    m->rows = rows;
    m->cols = cols;
    return MAT_ERR_NONE;
}

void matrix_free(matrix *m){
    free(m->data);
}

int matrix_copy(matrix *dest, matrix *src){
    if(dest->cols != src->cols || dest->rows != src->rows)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < src->rows; ++row){
        for(size_t col = 0; col < src->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src->data[MAT_IDX(src, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_zeros(matrix *m){
    for(size_t row = 0; row < m->rows; ++row){
        for(size_t col = 0; col < m->cols; ++col){
            m->data[MAT_IDX(m, row, col)] = 0;
        }
    }
    return MAT_ERR_NONE;
}

int matrix_ones(matrix *m){
    for(size_t row = 0; row < m->rows; ++row){
        for(size_t col = 0; col < m->cols; ++col){
            m->data[MAT_IDX(m, row, col)] = 1;
        }
    }
    return MAT_ERR_NONE;
}

int matrix_ident(matrix *m){
    for(size_t row = 0; row < m->rows; ++row){
        for(size_t col = 0; col < m->cols; ++col){
            if(row == col)
                m->data[MAT_IDX(m, row, col)] = 1;
            else
                m->data[MAT_IDX(m, row, col)] = 0;
        }
    }
    return MAT_ERR_NONE;
}

void matrix_print(matrix *m){
#ifdef MAT_EN_PRINT
    for(size_t row = 0; row < m->rows; ++row){
        for(size_t col = 0; col < m->cols; ++col){
            printf("%.2f\t", m->data[MAT_IDX(m, row, col)]);
        }
        printf("\n");
    }
#endif
}

int matrix_set_item(matrix *m, size_t row, size_t col, float item){
    if(row >= m->rows || col >= m->cols)
        return MAT_ERR_INDX;
    m->data[MAT_IDX(m, row, col)] = item;
    return MAT_ERR_NONE;
}

int matrix_set_row(matrix *m, size_t row, float *data){
    if(row >= m->rows)
        return MAT_ERR_INDX;
    for(size_t col = 0; col < m->cols; ++col){
        m->data[MAT_IDX(m, row, col)] = data[col];
    }
    return MAT_ERR_NONE;
}

int matrix_set_col(matrix *m, size_t col, float *data){
    if(col >= m->cols)
        return MAT_ERR_INDX;
    for(size_t row = 0; row < m->rows; ++row){
        m->data[MAT_IDX(m, row, col)] = data[row];
    }
    return MAT_ERR_NONE;
}

int matrix_get_item(float *item, matrix *m, size_t row, size_t col){
    if(row >= m->rows || col >= m->cols)
        return MAT_ERR_INDX;
    *item = m->data[MAT_IDX(m, row, col)];
    return MAT_ERR_NONE;
}

int matrix_get_row(float *data, matrix *m, size_t row){
    if(row >= m->rows)
        return MAT_ERR_INDX;
    for(size_t col = 0; col < m->cols; ++col){
        data[col] = m->data[MAT_IDX(m, row, col)];
    }
    return MAT_ERR_NONE;
}

int matrix_get_col(float *data, matrix *m, size_t col){
    if(col >= m->cols)
        return MAT_ERR_INDX;
    for(size_t row = 0; row < m->rows; ++row){
        data[row] = m->data[MAT_IDX(m, row, col)];
    }
    return MAT_ERR_NONE;
}

int matrix_add(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != src1->cols || dest->cols != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] + src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_sub(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != src1->cols || dest->cols != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] - src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_ew_mul(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != src1->cols || dest->cols != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] * src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_ew_div(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != src1->cols || dest->cols != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] / src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_sc_mul(matrix *dest, matrix *src, float scalar){
    if(dest->rows != src->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != src->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src->data[MAT_IDX(src, row, col)] * scalar;
        }
    }
    return MAT_ERR_NONE;
}

int matrix_sc_div(matrix *dest, matrix *src, float scalar){
    if(dest->rows != src->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != src->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src->data[MAT_IDX(src, row, col)] / scalar;
        }
    }
    return MAT_ERR_NONE;
}

int matrix_mul(matrix *dest, matrix *op1, matrix *op2){
    if(dest->rows != op1->rows)
        return MAT_ERR_SIZE;
    if(dest->cols != op2->cols)
        return MAT_ERR_SIZE;
    if(op1->cols != op2->rows)
        return MAT_ERR_SIZE;
    matrix_zeros(dest);
    for(size_t i = 0; i < op1->rows; ++i){
        for(size_t j = 0; j < op2->cols; ++j){
            for(size_t k = 0; k < op1->cols; ++k){
                dest->data[MAT_IDX(dest, i, j)] += op1->data[MAT_IDX(op1, i, k)] * op2->data[MAT_IDX(op2, k, j)];
            }
        }
    }
    return MAT_ERR_NONE;
}

int matrix_transpose(matrix *dest, matrix *src){
    if(dest->rows != src->cols || dest->cols != src->rows)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src->data[MAT_IDX(src, col, row)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_det(float *det, matrix *m){
    if(m->rows != m->cols)
        return MAT_ERR_SIZE;
    if(m->rows == 1){
        *det = m->data[MAT_IDX(m, 0, 0)];
    }else if(m->rows == 2){
        *det = m->data[MAT_IDX(m, 0, 0)] * m->data[MAT_IDX(m, 1, 1)] - m->data[MAT_IDX(m, 1, 0)] * m->data[MAT_IDX(m, 0, 1)];
    }else{
        *det = 0;
        for(size_t j1 = 0; j1 < m->rows; ++j1){
            matrix subm;
            matrix_init(&subm, m->rows - 1, m->rows - 1);
            for(size_t i = 1; i < m->rows; ++i){
                size_t j2 = 0;
                for(size_t j = 0; j < m->rows; ++j){
                    if(j == j1)
                        continue;
                    subm.data[MAT_IDX(&subm, i-1, j2)] = m->data[MAT_IDX(m, i, j)];
                    j2++;
                }
            }
            float subdet;
            matrix_det(&subdet, &subm);
            *det += powf(-1, j1 + 2.0) * m->data[MAT_IDX(m, 0, j1)] * subdet;
            matrix_free(&subm);
        }
    }
    return MAT_ERR_NONE;
}

int matrix_cofactor(matrix *dest, matrix *m){
    if(m->rows != m->cols)
        return MAT_ERR_SIZE;
    matrix tmp;
    matrix_init(&tmp, m->rows - 1, m->rows  -1);
    for(size_t j = 0; j < m->rows; ++j){
        for(size_t i = 0; i < m->rows; ++i){
            size_t i1 = 0;
            for(size_t ii = 0; ii < m->rows; ++ii){
                if(ii == i)
                    continue;
                size_t j1 = 0;
                for(size_t jj = 0; jj < m->rows; ++jj){
                    if(jj == j)
                        continue;
                    tmp.data[MAT_IDX(&tmp, i1, j1)] = m->data[MAT_IDX(m, ii, jj)];
                    j1++;
                }
                i1++;
            }
            float det;
            matrix_det(&det, &tmp);
            dest->data[MAT_IDX(dest, i, j)] = powf(-1.0, i + j + 2.0) * det;
        }
    }
    matrix_free(&tmp);
    return MAT_ERR_NONE;
}

int matrix_inv(matrix *dest, matrix *src){
    if(src->rows != src->cols)
        return MAT_ERR_SIZE;
    
    float det;
    matrix_det(&det, src);

    matrix cof, adj;
    matrix_init(&cof, src->rows, src->cols);
    matrix_init(&adj, src->rows, src->cols);
    matrix_cofactor(&cof, src);
    matrix_transpose(&adj, &cof);
    matrix_sc_div(dest, &adj, det);

    matrix_free(&adj);
    matrix_free(&cof);
    return MAT_ERR_NONE;
}
