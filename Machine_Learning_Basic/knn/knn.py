import math
from collections import Counter

def euclidean_distance(point1, point2):
    distance = 0.0
    for i in range(len(point1)):
        distance += (point1[i] - point2[i]) ** 2
    return math.sqrt(distance)

def get_neightbors(train_data, test_point, k):
    distances = []

    for train_point in train_data:
        distance = euclidean_distance(test_point, train_point[:-1])
        distances.append((train_point, distance))

    distances.sort(key=lambda x: x[1])


    neighbors = []

    for i in range(k):
        neighbors.append(distances[i][0])

    return neighbors


def predict_classification(neighbors):
    output_values = [neightbor[-1] for neightbor in neighbors]
    # print(Counter(output_values).most_common())
    prediction = Counter(output_values).most_common()[0][0]
    return prediction


def knn(train_data, test_data, k):
    neighbors = get_neightbors(train_data,test_point,k)
    prediction = predict_classification(neighbors)
    return prediction

if __name__ == '__main__':
    train_data = [
        [2.7, 2.5, 'A'],
        [1.0, 1.0, 'B'],
        [3.0, 3.5, 'A'],
        [0.5, 1.0, 'B'],
        [2.8, 2.9, 'A'],
        [0.6, 0.7, 'B']
    ]

    test_point = [1.5, 1.5]

    k = 3

    prediction = knn(train_data, test_point, k)
    print(f'The predicted class for test point {test_point} is {prediction}')