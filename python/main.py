import os
import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt  


def main():
    #CHANGE THESE ARGUMENTS IF NEEDED
    startframe = 1450
    endframe = 1450
    c_path = "../c++project/Debug/c++project.exe"
    draco_path = "../draco/build_dir/Debug"
    pointcloud_path = "../pointclouds/redandblack/redandblack/Ply/redandblack_vox10_"
    output_path = "../pointclouds/encoded/redandblack"
    max_threads = 4

    load_e_avg = np.array([])
    enc_avg = np.array([])
    save_e_avg = np.array([])
    load_d_avg = np.array([])
    dec_avg = np.array([])
    save_d_avg = np.array([])

    p2p_avg = np.array([])
    color_avg = np.array([])
    size_e_avg = np.array([])

    size_d_avg = np.array([])
    

    #TEST 1
    for i in range(10):
        os.system("powershell " + c_path + " -draco-path " + draco_path + " -i " + pointcloud_path + " -o " + output_path + " - sf " +str(startframe) + " -ef " + str(endframe) + " -threads 1  -test1 -q " + str(i))
        filename = open('../output.csv', 'r')
        file = csv.DictReader(filename)

        load_e =np.array([])
        enc = np.array([])
        save_e = np.array([])
        load_d = np.array([])
        dec = np.array([])
        save_d = np.array([])
        size_e = np.array([])

        size_d = np.array([])
        
        for col in file:

            load_e = np.insert(load_e, len(load_e), col['load encode'])
            enc = np.insert(enc, len(enc), col['encode'])
            save_e = np.insert(save_e, len(save_e), col['save encode'])
            load_d = np.insert(load_d, len(load_d), col['load decode'])
            dec = np.insert(dec, len(dec), col['decode'])
            save_d = np.insert(save_d, len(save_d), col['save decode'])
            size_e = np.insert(size_e, len(size_e), col['size encoded'])
            size_d = np.insert(size_d, len(size_d), col['size decoded'])


        load_e_avg = np.append(load_e_avg, np.average(load_e))
        enc_avg = np.append(enc_avg, np.average(enc))
        save_e_avg = np.append(save_e_avg, np.average(save_e))

        total_enc = np.array(load_e_avg + enc_avg + save_e_avg)

        load_d_avg = np.append(load_d_avg, np.average(load_d))
        dec_avg = np.append(dec_avg, np.average(dec))
        save_d_avg = np.append(save_d_avg, np.average(save_d))

        total_dec = np.array(load_d_avg + dec_avg + save_d_avg)

        size_e_avg = np.append(size_e_avg, np.average(size_e))
        size_d_avg = np.append(size_d_avg, np.average(size_d))


    #PRINTING
    x = np.array(range(0,10))

    plt.title("encode time / compression level")
    plt.plot(x, load_e_avg, label = "load time") 
    plt.plot(x, enc_avg, label = "encode time") 
    plt.plot(x, save_e_avg, label = "save time") 
    plt.plot(x, total_enc, label = "total encode time")
    plt.xticks(x)
    plt.xlabel('compression level') 
    plt.ylabel('time (ms)') 
    plt.legend()
    plt.savefig("encode time compression level 1.jpg")

    _, ax = plt.subplots()
    plt.title("encode time / compression level")
    ax.bar(x, load_e_avg, label = "load time")
    ax.bar(x, enc_avg, label = "encode time", bottom=load_e_avg)
    ax.bar(x, save_e_avg, label = "save time", bottom=load_e_avg + enc_avg)
    plt.xticks(x)
    ax.legend()
    plt.xlabel('compression level') 
    plt.ylabel('time (ms)') 
    plt.savefig("encode time compression level 2.jpg")

    plt.clf()
    plt.title("decode time / compression level")
    plt.plot(x, load_d_avg, label = "load time") 
    plt.plot(x, dec_avg, label = "decode time") 
    plt.plot(x, save_d_avg, label = "save time") 
    plt.plot(x, total_dec, label = "total decode time") 
    plt.xticks(x)
    plt.xlabel('compression level') 
    plt.ylabel('time (ms)') 
    plt.legend()
    plt.savefig("decode time compression level 1.jpg")

    _, ax = plt.subplots()
    plt.title("decode time / compression level")
    ax.bar(x, load_d_avg, label = "load time")
    ax.bar(x, dec_avg, label = "decode time", bottom=load_d_avg)
    ax.bar(x, save_d_avg, label = "save time", bottom=load_d_avg + dec_avg)
    plt.xticks(x)
    ax.legend()
    plt.xlabel('compression level') 
    plt.ylabel('time (ms)') 
    plt.savefig("decode time compression level 2.jpg")
    
    plt.clf()
    plt.title("encoded file size / compression level")
    plt.plot(x, size_e_avg/1000000, label = "encoded size") 
    plt.xlabel('compression level') 
    plt.ylabel('size (MB)') 
    plt.legend()
    plt.savefig("compression level_size_encoded.jpg")
    
    #TEST 2

    load_s_avg = np.array([])
    samp_avg = np.array([])
    save_s_avg = np.array([])
    load_e_avg = np.array([])
    enc_avg = np.array([])
    save_e_avg = np.array([])
    load_d_avg = np.array([])
    dec_avg = np.array([])
    save_d_avg = np.array([])
    p2p_avg = np.array([])
    color_avg = np.array([])
    size_e_avg = np.array([])
    size_d_avg = np.array([])
    points_avg = np.array([])


    for i in range(0, 10):
        os.system("powershell " + c_path + " -draco-path " + draco_path + " -i " + pointcloud_path + " -o " + output_path + " - sf " +str(startframe) + " -ef " + str(endframe) + " -threads 1 -test2 -q 0 -s " + str(i))
        filename = open('../output.csv', 'r')
        file = csv.DictReader(filename)

        load_s =np.array([])
        samp = np.array([])
        save_s = np.array([])
        load_e =np.array([])
        enc = np.array([])
        save_e = np.array([])
        load_d = np.array([])
        dec = np.array([])
        save_d = np.array([])
        p2p = np.array([])
        color = np.array([])
        size_e = np.array([])
        size_d = np.array([])
        points = np.array([])
        for col in file:

            load_s = np.insert(load_s, len(load_s), col['load sample'])
            samp = np.insert(samp, len(samp), col['sample'])
            save_s = np.insert(save_s, len(save_s), col['save sample'])
            load_e = np.insert(load_e, len(load_e), col['load encode'])
            enc = np.insert(enc, len(enc), col['encode'])
            save_e = np.insert(save_e, len(save_e), col['save encode'])
            load_d = np.insert(load_d, len(load_d), col['load decode'])
            dec = np.insert(dec, len(dec), col['decode'])
            save_d = np.insert(save_d, len(save_d), col['save decode'])
            p2p = np.insert(p2p, len(p2p), col['PSNR P2P'])
            color = np.insert(color, len(color), col['PSNR colors'])
            size_e = np.insert(size_e, len(size_e), col['size encoded'])
            size_d = np.insert(size_d, len(size_d), col['size decoded'])
            points = np.insert(points, len(points), col['points after sampling'])

        load_s_avg = np.append(load_s_avg, np.average(load_s))
        samp_avg = np.append(samp_avg, np.average(samp))
        save_s_avg = np.append(save_s_avg, np.average(save_s))

        total_samp = np.array(load_s_avg + samp_avg + save_s_avg)

        load_e_avg = np.append(load_e_avg, np.average(load_e))
        enc_avg = np.append(enc_avg, np.average(enc))
        save_e_avg = np.append(save_e_avg, np.average(save_e))

        total_enc = np.array(load_e_avg + enc_avg + save_e_avg)

        load_d_avg = np.append(load_d_avg, np.average(load_d))
        dec_avg = np.append(dec_avg, np.average(dec))
        save_d_avg = np.append(save_d_avg, np.average(save_d))

        total_dec = np.array(load_d_avg + dec_avg + save_d_avg)

        p2p_avg = np.append(p2p_avg, np.average(p2p))
        color_avg = np.append(color_avg, np.average(color))

        size_e_avg = np.append(size_e_avg, np.average(size_e))
        size_d_avg = np.append(size_d_avg, np.average(size_d))

        points_avg = np.append(points_avg, np.average(points))


    #PRINTING
    x = np.array(range(0,10))

    plt.clf()
    plt.title("sample time / leafsize")
    plt.plot(x, load_s_avg, label = "load time") 
    plt.plot(x, samp_avg, label = "sample time") 
    plt.plot(x, save_s_avg, label = "save time") 
    plt.plot(x, total_samp, label = "total sample time") 
    plt.xticks(x)
    plt.xlabel('leafsize') 
    plt.ylabel('time (ms)') 
    plt.legend()
    plt.savefig("sample time leafsize 1.jpg")

    _, ax = plt.subplots()
    plt.title("sample time / leafsize")
    ax.bar(x, load_s_avg, label = "load time")
    ax.bar(x, samp_avg, label = "sample time", bottom=load_s_avg)
    ax.bar(x, save_s_avg, label = "save time", bottom=load_s_avg + samp_avg)
    plt.xticks(x)
    ax.legend()
    plt.xlabel('leafsize') 
    plt.ylabel('time (ms)') 
    plt.savefig("sample time leafsize 2.jpg")

    plt.clf()
    plt.title("encode time / leafsize")
    plt.plot(x, load_e_avg, label = "load time") 
    plt.plot(x, enc_avg, label = "encode time") 
    plt.plot(x, save_e_avg, label = "save time") 
    plt.plot(x, total_enc, label = "total encode time") 
    plt.xticks(x)
    plt.xlabel('leafsize') 
    plt.ylabel('time (ms)') 
    plt.legend()
    plt.savefig("encode time leafsize 1.jpg")

    _, ax = plt.subplots()
    plt.title("encode time / leafsize")
    ax.bar(x, load_e_avg, label = "load time")
    ax.bar(x, enc_avg, label = "encode time", bottom=load_e_avg)
    ax.bar(x, save_e_avg, label = "save time", bottom=load_e_avg + enc_avg)
    plt.xticks(x)
    ax.legend()
    plt.xlabel('leafsize') 
    plt.ylabel('time (ms)') 
    plt.savefig("encode time leafsize 2.jpg")

    plt.clf()
    plt.title("decode time / leafsize")
    plt.plot(x, load_d_avg, label = "load time") 
    plt.plot(x, dec_avg, label = "decode time") 
    plt.plot(x, save_d_avg, label = "save time") 
    plt.plot(x, total_dec, label = "total decode time") 
    plt.xticks(x)
    plt.xlabel('leafsize') 
    plt.ylabel('time (ms)') 
    plt.legend()
    plt.savefig("decode time leafsize 1.jpg")

    _, ax = plt.subplots()
    plt.title("decode time / leafsize")
    ax.bar(x, load_d_avg, label = "load time")
    ax.bar(x, dec_avg, label = "decode time", bottom=load_d_avg)
    ax.bar(x, save_d_avg, label = "save time", bottom=load_d_avg + dec_avg)
    plt.xticks(x)
    ax.legend()
    plt.xlabel('leafsize') 
    plt.ylabel('time (ms)') 
    plt.savefig("decode time leafsize 2.jpg")
    
    plt.clf()
    plt.title("PSNR / leafsize")
    plt.plot(x, p2p_avg, label = "point to point") 
    plt.plot(x, color_avg, label = "colors") 
    plt.xticks(x)
    plt.xlabel('leafsize') 
    plt.ylabel('PSNR') 
    plt.legend()
    plt.savefig("PSNR leafsize.jpg")

    plt.clf()
    plt.title("encoded file size / leafsize")
    plt.plot(x, size_e_avg/1000000, label = "encoded size")
    plt.xticks(x)
    plt.xlabel('leafsize') 
    plt.ylabel('size (MB)') 
    plt.legend()
    plt.savefig("size encoded leafsize.jpg")

    plt.clf()
    plt.title("decoded file size / leafsize")
    plt.plot(x, size_d_avg/1000000, label = "decoded size")
    plt.xticks(x) 
    plt.xlabel('leafsize') 
    plt.ylabel('size (MB)') 
    plt.legend()
    plt.savefig("size decoded leafsize.jpg")

    plt.clf()
    plt.title("points before decoding / leafsize")
    plt.plot(x, points_avg)
    plt.xlabel('leafsize') 
    plt.xticks(x)
    plt.ylabel('points') 
    plt.legend()
    plt.savefig("points leafsize.jpg")
    endframe = 1454
    #test3

    os.system("powershell " + c_path + " -draco-path " + draco_path + " -i " + pointcloud_path + " -o " + output_path + " - sf " +str(startframe) + " -ef " + str(endframe) + " -test3 -max_threads " + str(max_threads))
    

    filename = open('../output.csv', 'r')
    file = csv.DictReader(filename)
    threads =np.array([])
    withouts = np.array([])
    w3 = np.array([])
    w5 = np.array([])
    w10 = np.array([])
    for col in file:
        threads = np.insert(threads, len(threads), col['threads'])
        withouts = np.insert(withouts, len(withouts), col['without sampling'])
        w3 = np.insert(w3, len(w3), col['leafsize = 3'])
        w5 = np.insert(w5, len(w5), col['leafsize = 5'])
        w10 = np.insert(w10, len(w10), col['leafsize = 10'])
    plt.clf()
    plt.title("threads / total send time")
    plt.plot(threads, withouts, label = "without sampling (compression level = 0)")
    plt.plot(threads, w3, label = "with sampling (compression level = 0, leafsize = 3)")
    plt.plot(threads, w5, label = "with sampling (compression level = 0, leafsize = 5)") 
    plt.plot(threads, w10, label = "with sampling (compression level = 0, leafsize = 10)")
    plt.xlabel('threads') 
    plt.ylabel('time (ms)') 
    plt.legend()
    plt.savefig("threads.jpg")


if __name__ == '__main__':
    
    class CustomHelpFormatter(argparse.HelpFormatter):
        def _format_action_invocation(self, action):
            if not action.option_strings or action.nargs == 0:
                return super()._format_action_invocation(action)
            default = self._get_default_metavar_for_optional(action)
            args_string = self._format_args(action, default)
            return ', '.join(action.option_strings) + '   ' + args_string

    parser = argparse.ArgumentParser(formatter_class=CustomHelpFormatter)


    args = parser.parse_args()

    main(**vars(args))
    end = input("to close press enter")