
def matrixMultiplication(m1,m2):
    """
    Multiplies two matrices together

    Args:
        m1 : Matrix 1
        m2 : Matrix 2

    Returns:
        result: The matrix result of the multiplication if possible
    """
    # Check if the matrices can be multiplied
    if len(m1[0]) != len(m2):
        print("Matrices cannot be multiplied")
        return None

    # Create a matrix to hold the result
    result = [[0 for x in range(len(m2[0]))] for y in range(len(m1))]

    # Multiply the two matrices
    for i in range(len(m1)):
        for j in range(len(m2[0])):
            for k in range(len(m2)):
                result[i][j] += m1[i][k] * m2[k][j]

    return result

def matrixMultiplicationVariable(m1,var1):
    """
    Multiplies two matrices together

    Args:
        m1 : Matrix 1
        var1 : Variable 1

    Returns:
        result: The matrix result of the multiplication if possible
    """
    # Create a matrix to hold the result
    result = [[0 for x in range(len(m1[0]))] for y in range(len(m1))]

    # Multiply the two matrices
    for i in range(len(m1)):
        for j in range(len(m1[0])):
            result[i][j] = m1[i][j] * var1

    return result


def matrixAddition(m1,m2):
    """
    Adds two matrices together

    Args:
        m1 : Matrix 1
        m2 : Matrix 2

    Returns:
        result : The matrix result of the addition if possible
    """
    # Check if the matrices can be added
    if len(m1) != len(m2) or len(m1[0]) != len(m2[0]):
        print("Matrices cannot be added")
        return None

    # Create a matrix to hold the result
    result = [[0 for x in range(len(m1[0]))] for y in range(len(m1))]

    # Add the two matrices
    for i in range(len(m1)):
        for j in range(len(m1[0])):
            result[i][j] = m1[i][j] + m2[i][j]

    return result

def matrixSubtraction(m1,m2):
    """
    Subtracts two matrices together

    Args:
        m1 : Matrix 1
        m2 : Matrix 2

    Returns:
        result : The matrix result of the subtraction if possible
    """
    # Check if the matrices can be subtracted
    if len(m1) != len(m2) or len(m1[0]) != len(m2[0]):
        print("Matrices cannot be subtracted")
        return None

    # Create a matrix to hold the result
    result = [[0 for x in range(len(m1[0]))] for y in range(len(m1))]

    # Subtract the two matrices
    for i in range(len(m1)):
        for j in range(len(m1[0])):
            result[i][j] = m1[i][j] - m2[i][j]

    return result

def matrixTranspose(m1):
    """
    Transforms a matrix

    Args:
        m1 : Matrix 1

    Returns:
        result : The matrix result of the transformation
    """
    # Create a matrix to hold the result
    result = [[0 for x in range(len(m1))] for y in range(len(m1[0]))]

    # Transform the matrix
    for i in range(len(m1)):
        for j in range(len(m1[0])):
            result[j][i] = m1[i][j]

    return result

def matrixInversion2x2(m1):
    """
    Inverts a 2x2 matrix

    Args:
        m1 : Matrix 1

    Returns:
        result : The matrix result of the inversion
    """
    # Check if the matrix is 2x2
    if len(m1) != 2 or len(m1[0]) != 2:
        print("Matrix is not 2x2")
        return None

    # Create a matrix to hold the result
    result = [[0 for x in range(2)] for y in range(2)]
    
    # calculate the determinant
    det = matrixDeterminante2x2(m1)
    
    # Invert the matrix
    result[0][0] = m1[1][1] / det
    result[1][1] = m1[0][0] / det
    result[0][1] = -m1[0][1] / det
    result[1][0] = -m1[1][0] / det
    
    return result
    
    
    
def matrixDeterminante2x2(m1):
    """
    Calculates the determinant of a matrix

    Args:
        m1 : Matrix 1

    Returns:
        result : The determinant of the matrix
    """
    # Check if the matrix is 2x2
    if len(m1) != 2 or len(m1[0]) != 2:
        print("Matrix is not 2x2")
        return None

    # Calculate the determinant
    result = m1[0][0] * m1[1][1] - m1[0][1] * m1[1][0]

    return result