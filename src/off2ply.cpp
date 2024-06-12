#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

struct Vertex {
    float x, y, z;
};

struct Face {
    std::vector<int> indices;
};

bool readOFFFile(const std::string& filename, std::vector<Vertex>& vertices, std::vector<Face>& faces) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Unable to open OFF file: " << filename << std::endl;
        return false;
    }

    std::string line;
    std::getline(infile, line);
    if (line != "OFF") {
        std::cerr << "Not a valid OFF file: " << filename << std::endl;
        return false;
    }

    int numVertices, numFaces, numEdges;
    infile >> numVertices >> numFaces >> numEdges;

    vertices.resize(numVertices);
    faces.resize(numFaces);

    for (int i = 0; i < numVertices; ++i) {
        infile >> vertices[i].x >> vertices[i].y >> vertices[i].z;
    }

    for (int i = 0; i < numFaces; ++i) {
        int numIndices;
        infile >> numIndices;
        faces[i].indices.resize(numIndices);
        for (int j = 0; j < numIndices; ++j) {
            infile >> faces[i].indices[j];
        }
    }

    return true;
}

bool writePLYFile(const std::string& filename, const std::vector<Vertex>& vertices, const std::vector<Face>& faces) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Unable to open PLY file: " << filename << std::endl;
        return false;
    }

    outfile << "ply" << std::endl;
    outfile << "format ascii 1.0" << std::endl;
    outfile << "element vertex " << vertices.size() << std::endl;
    outfile << "property float x" << std::endl;
    outfile << "property float y" << std::endl;
    outfile << "property float z" << std::endl;
    outfile << "element face " << faces.size() << std::endl;
    outfile << "property list uchar int vertex_indices" << std::endl;
    outfile << "end_header" << std::endl;

    for (const auto& vertex : vertices) {
        outfile << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
    }

    for (const auto& face : faces) {
        outfile << face.indices.size();
        for (const auto& index : face.indices) {
            outfile << " " << index;
        }
        outfile << std::endl;
    }

    return true;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.off> <output.ply>" << std::endl;
        return 1;
    }

    std::vector<Vertex> vertices;
    std::vector<Face> faces;

    if (!readOFFFile(argv[1], vertices, faces)) {
        return 1;
    }

    if (!writePLYFile(argv[2], vertices, faces)) {
        return 1;
    }

    std::cout << "Conversion completed successfully!" << std::endl;
    return 0;
}
