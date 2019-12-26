#include <opencv2/imgproc/imgproc.hpp>

void DrawLine(cv::Mat &img, cv::Point start, cv::Point end)
{
    int thickness = 5;
    int type = 8;
    line(img,
         start,
         end,
         cv::Scalar(255, 255, 255),
         thickness,
         type);
}

void write_positions_to_image(std::vector<std::vector<double>> &positions)
{
    cv::Mat contour_img = cv::Mat::zeros(w, w, CV_8UC3);
    int len = positions.size();
    std::vector<cv::Point> points = std::vector<cv::Point>();
    for (int i = 0; i < len; i++)
    {
        std::vector<double> pos = positions[i];
        cv::Point p = cv::Point(pos[0] + (w / 2), pos[1] + (w / 2));
        points.push_back(p);
    }

    for (int i = 0; i < len - 1; i++)
    {
        DrawLine(contour_img, points[i], points[i + 1]);
    }

    cv::imwrite("contour.jpg", contour_img);
}

void plot_contour()
{
    std::vector<double> curr_loc = homogenous_coord_init(0.0, 0.0, 1.0);
    static std::vector<std::vector<double>> positions = std::vector<std::vector<double>>();

    positions.push_back(curr_loc);
    int len = distances.size();
    double u1 = 1.0;
    double u2 = 0.0;
    for (int i = 0; i < len; i++)
    {
        double theta = angles[i] * (3.141592653 / 180.0);
        double d = distances[i];
        std::vector<double> d_vec = homogenous_coord_init(d * u1,
                                                          d * u2,
                                                          1.0);
        curr_loc[0] += d_vec[0];
        curr_loc[1] += d_vec[1];
        cout << "***********" << endl;
        cout << "d: " << d << endl;
        cout << "angles: " << theta << endl;
        cout << "u1: " << u1 << ", u2: " << u2 << endl;
        cout << curr_loc[0] << "," << curr_loc[1] << endl;
        positions.push_back(curr_loc);
        float temp_u1 = u1 * cos(theta) - u2 * sin(theta);
        float temp_u2 = u1 * sin(theta) + u2 * cos(theta);
        u1 = temp_u1;
        u2 = temp_u2;
    }
    write_positions_to_image(positions);
}

void main()