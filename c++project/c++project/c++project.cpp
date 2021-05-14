#include <iostream>
#include <fstream>
#include <cstdint>
#include <filesystem>
#include <pthread.h>
#include <string>
#include <mutex>
#include <ctime>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace std::chrono;
namespace fs = std::filesystem;
int current_frame;
int threads_running = 0;
std::mutex frame_lock;
std::mutex cv_m;
std::condition_variable cv;
std::string path_to_draco_encoder;
std::string path_to_draco_decoder;
std::string path_to_mpeg_psnr;
std::string path_to_ply_files;
std::string path_encoded_files;
std::string path_decoded_files;
std::string path_sampled_files;
std::string path_sampled_encoded_files;
std::string path_sampled_decoded_files;
int startframe = 1450;
int endframe = 1749;
int num_threads = 1;
int max_threads = 4;
int top_level_quality = 7;
int top_level_leafsize = 0;

//testing
bool test1 = false;
bool test2 = false;
bool test3 = false;
bool test4 = false;

//structs used to pass arguments to the threads
struct encode_thread_args {
    int last_frame;
    int quality;
    string input;
    string output;
    encode_thread_args(string input, string output, int last_frame, int quality = 7)
    {
        this->input = input;
        this->output = output;
        this->last_frame = last_frame;
        this->quality = quality;
    }
};

struct decode_thread_args {
    int last_frame;
    string input;
    string output;
    decode_thread_args(string input, string output, int last_frame)
    {
        this->input = input;
        this->output = output;
        this->last_frame = last_frame;
    }
};

struct downsize_thread_args {
    int last_frame;
    float leafsize;
    string input;
    string output;
    downsize_thread_args(string input, string output, int last_frame, float leafsize = 1)
    {
        this->input = input;
        this->output = output;
        this->last_frame = last_frame;
        this->leafsize = leafsize;
    }
};

//encodes one frame with given quality
//thread_number only used with parallelization
void encode_frame(string input, string output, int frame_number = startframe, int quality = 7, bool print = false) {
    if (quality < 0 or quality > 10) {
        cout << "quality value " << quality << " is out of bounds" << endl;
        cout << "quality value should be between 0 and 10" << endl;
        return;
    }
    if (frame_number < startframe or frame_number > endframe) {
        cout << "frame_number value " << frame_number << " is out of bounds" << endl;
        cout << "frame_number value should be between " + std::to_string(startframe) + "and " + std::to_string(endframe) << endl;
        return;
    }
    string powershell;
#ifdef _WIN32
    powershell += "powershell ";
#endif
    powershell += path_to_draco_encoder + " -i " + input + std::to_string(frame_number) + ".ply -o " + output + std::to_string(frame_number) + ".drc -point_cloud -cl " + std::to_string(quality);
    if (print) {
        powershell += " -print";
    }
    system(powershell.c_str());
}

//decodes one frame
//thread_number only used with parallelization
void decode_frame(string input, string output, int frame_number = startframe, bool print = false) {
    if (frame_number < startframe or frame_number > endframe) {
        cout << "frame_number value " << frame_number << " is out of bounds" << endl;
        cout << "frame_number value should be between " + std::to_string(startframe) + "and " + std::to_string(endframe) << endl;
        return;
    }
    //TODO zou handig zijn als draco niet steeds zijn eigen output printen
    string powershell;
#ifdef _WIN32
    powershell += "powershell ";
#endif
    powershell += path_to_draco_decoder + " -i " + input + std::to_string(frame_number) + ".drc -o " + output + std::to_string(frame_number) + ".ply";
    if (print) {
        powershell += " -print";
    }
    system(powershell.c_str());

}

//samples one frame
void downsize(string input, string output, int frame_number = startframe, float leaf_size = 1, bool print = false) {
    high_resolution_clock::time_point start_time;
    high_resolution_clock::time_point end_time;

    //load
    start_time = high_resolution_clock::now();
    if (frame_number < startframe or frame_number > endframe) {
        cout << "frame_number value " << frame_number << " is out of bounds" << endl;
        cout << "frame_number value should be between " + std::to_string(startframe) + "and " + std::to_string(endframe) << endl;
        return;
    }
    string inp = input + std::to_string(frame_number) + ".ply";
    string outp = output + std::to_string(frame_number) + ".ply";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(inp, *cloud_in) == -1)
    {
        PCL_ERROR("Could not read the file");
        return;
    }

    end_time = high_resolution_clock::now();
    duration<double, std::milli> load_time = end_time - start_time;

    //sample
    start_time = high_resolution_clock::now();
    pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
    voxgrid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxgrid.setInputCloud(cloud_in);
    voxgrid.filter(*cloud_out);
    end_time = high_resolution_clock::now();
    duration<double, std::milli> sample_time = end_time - start_time;

    //save
    start_time = high_resolution_clock::now();
    pcl::io::savePLYFile(outp, *cloud_out);
    end_time = high_resolution_clock::now();
    duration<double, std::milli> save_time = end_time - start_time;
    cout << "sampled pointcloud saved to " + outp + "(" << load_time.count() << " ms to load, " << sample_time.count() << " ms to sample and  " << save_time.count() << " ms to save to a file.)" << endl;

    //print
    if (print) {
        ofstream myfile("C:/Users/isarl/Documents/VOP/output.csv",
            ios::out | ios::app);
        myfile << "," << load_time.count() << "," << sample_time.count() << ","
            << save_time.count();
        myfile.close();
    }
}

void calculatePSNR(string input_a, string input_b, int frame_number = startframe, bool print = false) {

    string inp = input_a + std::to_string(frame_number) + ".ply";
    string outp = input_b + std::to_string(frame_number) + ".ply";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(inp, *cloud_a) == -1)
    {
        PCL_ERROR("Could not read the file");
        return;
    }
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(outp, *cloud_b) == -1)
    {
        PCL_ERROR("Could not read the file");
        return;
    }

    pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_b);

    //suma = a compared to b, sumb = b compared to a
    //rgb is symmetric

    long float suma = 0;
    long float r = 0;
    long float g = 0;
    long float b = 0;


    for (pcl::PointXYZRGB pointa : *cloud_a) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree->nearestKSearch(pointa, 1, nn_indices, nn_dists);
        suma += pow(nn_dists[0], 2);
        r += pow(pointa.r - cloud_b->points[nn_indices[0]].r, 2);
        g += pow(pointa.g - cloud_b->points[nn_indices[0]].g, 2);
        b += pow(pointa.b - cloud_b->points[nn_indices[0]].b, 2);
    }

    long float sumb = 0;
    tree->setInputCloud(cloud_a);
    for (pcl::PointXYZRGB pointa : *cloud_b) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree->nearestKSearch(pointa, 1, nn_indices, nn_dists);
        sumb += pow(nn_dists[0], 2);
    }

    long float sum = max(suma, sumb);
    int k = cloud_a->size();
    long float d_sym = sum / k;

    r = r /k;
    g = g /k;
    b = b /k;

    float avg = (r + g + b) / 3;
    float PSNR_color = -1;
    // if avg = 0 PSNR should be infinity, we represend this with a value of -1
    if (avg != 0) {
        PSNR_color = 10 * log((256 * 256) / (avg));
    }

    float max_x = -1;
    float max_y = -1;
    float max_z = -1;
    // calculate max x y z values
    for (pcl::PointXYZRGB pointb : *cloud_b){
        if (pointb.x > max_x || max_x == -1) max_x = pointb.x;
        if (pointb.y > max_y || max_y == -1) max_y = pointb.y;
        if (pointb.z > max_z || max_z == -1) max_z = pointb.z;
    }
    float max = sqrt(max_x* max_x + max_y* max_y + max_z* max_z);
    float PSNR_p2p = 10 * log((max * max) / d_sym);
    cout <<"PSNR PointToPoint : "<< PSNR_p2p;
    cout << " & colors : " << PSNR_color << endl;
    if (print) {
        ofstream myfile("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
        myfile << "," << PSNR_p2p << "," << PSNR_color << ",";
        myfile.close();
    }
    return;
}
//counts the points of a pointcloud
long countpoints(string input, int frame_number = startframe) {

    string inp = input + std::to_string(frame_number) + ".ply";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(inp, *cloud_in) == -1)
    {
        PCL_ERROR("Could not read the file");
        return 0;
    }
    return cloud_in->size();
}

//encodes a series of frames without parallelization
void encode_frames(string input, string output, int start_frame = startframe, int last_frame = endframe, int quality = 7)
{
    if (start_frame < startframe) {
        cout << "last_frame is set to low" << endl;
        return;
    }
    if (last_frame > endframe) {
        cout << "last_frame is set to high" << endl;
        return;
    }
    for (int i = startframe; i < last_frame; i++)
    {
        encode_frame(input, output, i, quality);
    }
}

//decodes a series of frames without parallelization
void decode_frames(string input, string output, int start_frame = startframe, int last_frame = endframe)
{
    if (start_frame < startframe) {
        cout << "last_frame is set to low" << endl;
        return;
    }
    if (last_frame > endframe) {
        cout << "number_of_frames is set to high" << endl;
        return;
    }
    for (int i = startframe; i < last_frame; i++)
    {
        decode_frame(input, output, i);
    }
}

//downsize a series of frames without parallelization
void downsize_frames(string input, string output, int start_frame = startframe, int last_frame = endframe, float leaf_size = 1) {
    if (start_frame < startframe) {
        cout << "last_frame is set to low" << endl;
        return;
    }
    if (last_frame > endframe) {
        cout << "number_of_frames is set to high" << endl;
        return;
    }
    for (int i = startframe; i < last_frame; i++)
    {
        downsize(input, output, i = startframe, leaf_size);
    }
}
//calculates the PSNR of a series of frames
void calculatePSNRs(string input, string output, int start_frame = startframe, int last_frame = endframe) {

    if (start_frame < startframe) {
        cout << "last_frame is set to low" << endl;
        return;
    }
    if (last_frame > endframe) {
        cout << "number_of_frames is set to high" << endl;
        return;
    }
    for (int i = startframe; i < last_frame; i++)
    {
        calculatePSNR(input, output, i);
    }
}

//encodes and decodes a series of frames without parallelization
void send_frames(string input, string encoder_ouput, string output, int start_frame = startframe, int last_frame = endframe, int quality = 7)
{
    encode_frames(input, encoder_ouput, start_frame, last_frame, quality);
    decode_frames(encoder_ouput, output, start_frame, last_frame);
}

//samples, encodes and decodes a series of frames without parallelization
void send_sampled_frames(string input, string sampler_output, string encoder_ouput, string output, int start_frame = startframe, int last_frame = endframe, float leaf_size = 1, int quality = 7)
{
    downsize_frames(input, sampler_output, start_frame, last_frame, leaf_size);
    send_frames(sampler_output, encoder_ouput, output, start_frame, last_frame, quality);
}

//runs a thread that will encode frames
void* Encode_frames_thread(void* argp) {
    struct encode_thread_args* args = (struct encode_thread_args*)argp;
    frame_lock.lock();
    while (current_frame < args->last_frame) {
        int temp = current_frame;
        current_frame++;
        frame_lock.unlock();
        encode_frame(args->input, args->output, temp, args->quality);
        frame_lock.lock();
    }
    frame_lock.unlock();
    threads_running--;
    cv.notify_one();
    pthread_exit(NULL);
    return NULL;
}

//runs a thread that will decode frames
void* decode_frames_thread(void* argp) {
    struct decode_thread_args* args = (struct decode_thread_args*)argp;
    frame_lock.lock();
    while (current_frame < args->last_frame) {
        int temp = current_frame;
        current_frame++;
        frame_lock.unlock();
        decode_frame(args->input, args->output, temp);
        frame_lock.lock();
    }
    frame_lock.unlock();
    threads_running--;
    cv.notify_one();
    pthread_exit(NULL);
    return NULL;
}

//runs a thread that will downsize frames
void* downsize_frames_thread(void* argp) {
    struct downsize_thread_args* args = (struct downsize_thread_args*)argp;
    frame_lock.lock();
    while (current_frame < args->last_frame) {
        int temp = current_frame;
        current_frame++;
        frame_lock.unlock();
        downsize(args->input, args->output, temp, args->leafsize);
        frame_lock.lock();
    }
    frame_lock.unlock();
    threads_running--;
    cv.notify_one();
    pthread_exit(NULL);
    return NULL;
}

//starts enough threads and uses them to encode
void Encode_frames_parrallel(string input, string output, int start_frame = startframe, int last_frame = endframe, int quality = 7) {
    pthread_t* threads = new pthread_t[num_threads];
    current_frame = start_frame;
    for (int i = 0; i < num_threads; i++) {
        struct encode_thread_args* args = new encode_thread_args(input, output, last_frame, quality);
        threads_running++;
        pthread_create(&threads[i], NULL, Encode_frames_thread, args);
    }
}

//starts enough threats and uses them to decode
void decode_frames_parrallel(string input, string output, int start_frame = startframe, int last_frame = endframe) {
    pthread_t* threads = new pthread_t[num_threads];
    current_frame = start_frame;
    for (int i = 0; i < num_threads; i++) {
        struct decode_thread_args* args = new decode_thread_args(input, output, last_frame);
        threads_running++;
        pthread_create(&threads[i], NULL, decode_frames_thread, args);
    }
}

//starts enough threats and uses them to downsize
void downsize_frames_parrallel(string input, string output, int start_frame = startframe, int last_frame = endframe, float leafsize = 1) {
    pthread_t* threads = new pthread_t[num_threads];
    current_frame = start_frame;
    for (int i = 0; i < num_threads; i++) {
        struct downsize_thread_args* args = new downsize_thread_args(input, output, last_frame, leafsize);
        threads_running++;
        pthread_create(&threads[i], NULL, downsize_frames_thread, args);
    }
}

//wait until threads_running = 0
void wait_for_thread() {
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait(lk, [] {return threads_running == 0;});
}

//encodes and decodes frames using parallelization
void Send_frames_parrallel(string input, string encoder_output, string output, int start_frame = startframe, int last_frame = endframe, int quality = 7) {
    Encode_frames_parrallel(input, encoder_output, start_frame, last_frame, quality);
    wait_for_thread();
    decode_frames_parrallel(encoder_output, output, start_frame, last_frame);
    wait_for_thread();
}

//samples encodes and decodes frames using parallelization
void Send_frames_parrallel(string input, string sampler_output, string encoder_output, string output, int start_frame = startframe, int last_frame = endframe, int quality = 7, float leafsize = 5) {
    downsize_frames_parrallel(input, sampler_output, start_frame, last_frame, leafsize);
    wait_for_thread();
    Encode_frames_parrallel(sampler_output, encoder_output, start_frame, last_frame, quality);
    wait_for_thread();
    decode_frames_parrallel(encoder_output, output, start_frame, last_frame);
    wait_for_thread();
}


void Usage() {
    printf("Usage: draco_multi [options]\n");
    printf("\n");
    printf("Main options:\n");
    printf("  -h | -?                               show help.\n");
    printf("  -draco-path <path_to_draco>           path to draco build files.\n");
    printf("  -mpeg-path <path_to_draco>            path to mpeg build files.\n");
    printf("  -i <input>                            path to ply input files (including filename).\n");
    printf("  -o <output path>                      path to output files (inluding filename).\n");
    printf("  -q <quality>                          draco quantization quality.\n");
    printf("  -sf <startframe>                      start frame number.\n");
    printf("  -ef <endframe>                        end frame number.\n");
    printf("  -threads <num_threads>                number of threads to use. Standard is 1.\n");
}

int filesize(string filename, int frame_number = startframe, bool ply = true)
{
    fs::path file;
    if (ply) {
        file = filename + std::to_string(frame_number) + ".ply";
    }
    else {
        file = filename + std::to_string(frame_number) + ".drc";
    }

    int fileSize = fs::file_size(file);

    return fileSize;
    
}

int main(int argc, char** argv) {

    int argc_check = argc - 1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp("-h", argv[i]) || !strcmp("-?", argv[i])) {
            Usage();
            return 0;
        }
        if (!strcmp("-draco-path", argv[i]) && i < argc_check) {
            std::string path_to_draco = argv[++i];
            path_to_draco_decoder = path_to_draco + "/draco_decoder";
            path_to_draco_encoder = path_to_draco + "/draco_encoder";
#ifdef _WIN32
            path_to_draco_decoder += ".exe";
            path_to_draco_encoder += ".exe";
#endif
        }
        if (!strcmp("-mpeg-path", argv[i]) && i < argc_check) {
            path_to_mpeg_psnr = argv[++i];
            path_to_mpeg_psnr += "/PccAppMetrics";
#ifdef _WIN32
            path_to_mpeg_psnr += ".exe";
#endif
        }
        else if (!strcmp("-i", argv[i]) && i < argc_check) {
            path_to_ply_files = argv[++i];
        }
        else if (!strcmp("-o", argv[i]) && i < argc_check) {
            std::string output_path = argv[++i];
            path_encoded_files = output_path + "_enc_";
            path_decoded_files = output_path + "_dec_";
            path_sampled_files = output_path + "_sampl_";
            path_sampled_encoded_files = output_path + "_sampl_enc_";
            path_sampled_decoded_files = output_path + "_sampl_dec_";
        }
        else if (!strcmp("-q", argv[i]) && i < argc_check) {
            top_level_quality = std::stoi(argv[++i]);
        }
        else if (!strcmp("-s", argv[i]) && i < argc_check) {
            top_level_leafsize = std::stoi(argv[++i]);
        }
        else if (!strcmp("-sf", argv[i]) && i < argc_check) {
            startframe = std::stoi(argv[++i]);
        }
        else if (!strcmp("-ef", argv[i]) && i < argc_check) {
            endframe = std::stoi(argv[++i]);
        }
        else if (!strcmp("-test1", argv[i])) {
            test1 = true;
        }
        else if (!strcmp("-test2", argv[i])) {
            test2 = true;
        }
        else if (!strcmp("-test3", argv[i])) {
            test3 = true;
        }
        else if (!strcmp("-test4", argv[i])) {
            test4 = true;
        }
        else if (!strcmp("-threads", argv[i]) && i < argc_check) {
            num_threads = std::stoi(argv[++i]);
        }
        else if (!strcmp("-max_threads", argv[i]) && i < argc_check) {
            num_threads = std::stoi(argv[++i]);
        }
    }
    if (path_to_draco_encoder.empty() || path_to_draco_decoder.empty() || path_to_ply_files.empty() || path_encoded_files.empty() || path_decoded_files.empty()) {
        Usage();
        return -1;
    }

    ofstream myfile("C:/Users/isarl/Documents/VOP/output.csv");
    myfile.close();

    if (test1) {
        myfile.open("C:/Users/isarl/Documents/VOP/output.csv");
        myfile << "Frame,load encode,encode,save encode,load decode,decode,save decode,size encoded,size decoded" << endl;
        myfile.close();
        for (int i = startframe; i <= endframe; i++)
        {
            myfile.open("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
            myfile << i;
            myfile.close();
            encode_frame(path_to_ply_files, path_encoded_files, i, top_level_quality, true);
            decode_frame(path_encoded_files, path_decoded_files, i, true);
            myfile.open("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
            myfile << filesize(path_encoded_files, startframe, false) << "," << filesize(path_decoded_files, startframe) << endl;
            myfile.close();
        }
    }

    if (test2) {
        
        myfile.open("C:/Users/isarl/Documents/VOP/output.csv");
        myfile << "Frame,load sample,sample,save sample,load encode,encode,save encode,load decode,decode,save decode,PSNR P2P,PSNR colors,size encoded,size decoded,points after sampling" << endl;
        myfile.close();
        for (int i = startframe; i <= endframe; i++)
        {
            myfile.open("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
            myfile << i;
            myfile.close();
            if (top_level_leafsize != 0) {
                downsize(path_to_ply_files, path_sampled_files, i, top_level_leafsize, true);
                encode_frame(path_sampled_files, path_encoded_files, i, 0, true);
                decode_frame(path_encoded_files, path_decoded_files, i, true);
                calculatePSNR(path_to_ply_files, path_decoded_files, i, true);
                myfile.open("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
                myfile << filesize(path_encoded_files, startframe, false) << "," << filesize(path_decoded_files, startframe) << ",";
                myfile << countpoints(path_sampled_files) << endl;
                myfile.close();
            }
            else {
                myfile.open("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
                myfile << "," << 0 << "," << 0 << "," << 0;
                myfile.close();
                encode_frame(path_to_ply_files, path_encoded_files, i, 0, true);
                decode_frame(path_encoded_files, path_decoded_files, i, true);
                calculatePSNR(path_to_ply_files, path_decoded_files, i, true);
                myfile.open("C:/Users/isarl/Documents/VOP/output.csv", ios::out | ios::app);
                myfile << filesize(path_encoded_files, startframe, false) << "," << filesize(path_decoded_files, startframe) << ",";
                myfile << countpoints(path_to_ply_files) << endl;
                myfile.close();
            }
        }
    }
    if (test3) {
        high_resolution_clock::time_point start_time;
        high_resolution_clock::time_point end_time;
        start_time = high_resolution_clock::now();
        myfile.open("C:/Users/isarl/Documents/VOP/output.csv");
        myfile << "threads,without sampling,leafsize = 3,leafsize = 5,leafsize = 10" << endl;
        //cout << max_threads << endl;
        for (int i = 1; i <= max_threads; i++)
        {
            num_threads = i;
            start_time = high_resolution_clock::now();
            Send_frames_parrallel(path_to_ply_files, path_encoded_files, path_decoded_files, startframe, endframe, 0);
            end_time = high_resolution_clock::now();
            duration<double, std::milli> wh = (end_time - start_time) / (endframe - startframe);
            start_time = high_resolution_clock::now();
            Send_frames_parrallel(path_to_ply_files, path_sampled_files, path_encoded_files, path_decoded_files, startframe, endframe, 0, 3);
            end_time = high_resolution_clock::now();
            duration<double, std::milli> w3 = (end_time - start_time) / (endframe - startframe);
            start_time = high_resolution_clock::now();
            Send_frames_parrallel(path_to_ply_files, path_sampled_files, path_encoded_files, path_decoded_files, startframe, endframe, 0, 5);
            end_time = high_resolution_clock::now();
            duration<double, std::milli> w5 = (end_time - start_time) / (endframe - startframe);
            start_time = high_resolution_clock::now();
            Send_frames_parrallel(path_to_ply_files, path_sampled_files, path_encoded_files, path_decoded_files, startframe, endframe, 0, 10);
            end_time = high_resolution_clock::now();
            duration<double, std::milli> w10 = (end_time - start_time) / (endframe - startframe);
            start_time = high_resolution_clock::now();
            Send_frames_parrallel(path_to_ply_files, path_sampled_files, path_encoded_files, path_decoded_files, startframe, endframe, 0, 15);
            end_time = high_resolution_clock::now();
            myfile << num_threads << "," << wh.count() << "," << w3.count() << "," << w5.count() << "," << w10.count() << endl;
        }
    }

    if (!(test1 || test2 || test3 || test4))
    {
        if (top_level_leafsize == 0) {
            Send_frames_parrallel(path_to_ply_files, path_encoded_files, path_decoded_files, startframe, endframe, top_level_quality);
        }
        else
        {
            Send_frames_parrallel(path_to_ply_files, path_sampled_files,path_encoded_files, path_decoded_files, startframe, endframe, top_level_quality, top_level_leafsize);
        }
    }
}
