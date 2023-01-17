#include <fstream>
#include <vector>
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

// Read and write csv data

class CSVData
{
    public:
    Eigen::MatrixXf data;
    std::string filename;

    CSVData(std::string filename_)
    {
        filename = filename_;
    }

    CSVData(std::string filename_, Eigen::MatrixXf data_)
    {
        filename = filename_;
        data = data_;
    }

    void writeToCSVfile()
    {
        std::ofstream file(filename.c_str());
        file << data.format(CSVFormat);
        file.close();
    }

    Eigen::MatrixXf readFromCSVfile()
    {
        std::vector<float> matrixEntries;
        std::ifstream matrixDataFile(filename);
        std::string matrixRowString;
        std::string matrixEntry;
        int matrixRowNumber = 0;
    
        while (getline(matrixDataFile, matrixRowString))
        {
            std::stringstream matrixRowStringStream(matrixRowString);
            while (getline(matrixRowStringStream, matrixEntry, ','))
            {
                matrixEntries.push_back(stod(matrixEntry));
            }
            matrixRowNumber++;
        }
        
        return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
    }
};