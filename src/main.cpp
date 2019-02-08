/*
    Copyright 2018 Brick

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software
    and associated documentation files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge, publish, distribute,
    sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
    BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <chrono>
#include <filesystem>

#include <mutex>
#include <condition_variable>
#include <future>
#include <queue>

#include <mem/mem.h>
#include <mem/pattern.h>

#include <fmt/format.h>
#include <mmaplib.h>

std::queue<std::pair<std::string, mmaplib::MemoryMappedFile>> FilesQueue;
std::mutex FilesMutex;
std::condition_variable FilesProduced;
std::condition_variable FilesConsumed;

std::size_t FilesLength = 0;
bool FilesDone = false;

std::mutex PrintLock;

template <typename... Args>
void Print(Args&&... args)
{
    std::unique_lock<std::mutex> lock(PrintLock);

    fmt::print(std::forward<Args>(args)...);
    fmt::print("\n");
}

void Producer(const std::string& directory)
{
    for (const std::filesystem::directory_entry& entry : std::filesystem::recursive_directory_iterator(directory))
    {
        if (entry.is_regular_file())
        {
            std::string path = entry.path().string();

            try
            {
                std::unique_lock<std::mutex> lock(FilesMutex);

                FilesConsumed.wait(lock, [ ]
                {
                    return (FilesLength < (1024 * 1024 * 128))
                        && (FilesQueue.size() < 16);
                });

                lock.unlock();

                mmaplib::MemoryMappedFile file(path.c_str());

                if (file.is_open())
                {
                    lock.lock();

                    FilesLength += file.size();
                    FilesQueue.emplace(std::move(path), std::move(file));

                    lock.unlock();
                    FilesProduced.notify_one();
                }
                else
                {
                    // Print("Failed to open {0}", path);
                }
            }
            catch (const std::exception& ex)
            {
                Print("Exception opening {0}: {1}", path, ex.what());
            }
        }
    }

    std::unique_lock<std::mutex> lock(FilesMutex);
    FilesDone = true;
    FilesProduced.notify_one();
}

void Consumer(const std::string& needle)
{
    mem::pattern pattern(needle.c_str(), nullptr, needle.size());
    mem::default_scanner scanner(pattern);

    std::size_t total_bytes = 0;
    std::size_t total_files = 0;

    std::unique_lock<std::mutex> lock(FilesMutex);

    while (true)
    {
        while (!FilesQueue.empty())
        {
            std::size_t file_size = 0;

            {
                std::pair<std::string, mmaplib::MemoryMappedFile> current = std::move(FilesQueue.front());
                FilesQueue.pop();

                lock.unlock();

                mem::region scan_range { current.second.data(), current.second.size() };

                for (mem::pointer addr : pattern.scan_all(scan_range, scanner))
                {
                    fmt::print("{0} : 0x{1:X}\n", current.first, addr - scan_range.start);
                }

                total_bytes += scan_range.size;
                total_files += 1;

                if (!(total_files & 0xFF))
                {
                    fmt::print("Total: {0} files, 0x{1:X} bytes ({2:.3f} GB)\n", total_files, total_bytes, (total_bytes / 1073741824.0));
                }

                lock.lock();

                file_size = current.second.size();
            }

            FilesLength -= file_size;
            FilesConsumed.notify_one();
        }

        if (FilesDone)
            break;

        FilesProduced.wait(lock);
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        fmt::print("Need 3 arguments\n");

        return 1;
    }

    using stopwatch = std::chrono::high_resolution_clock;

    auto begin_time = stopwatch::now();

    std::thread prod(Producer, argv[1]), cons(Consumer, argv[2]);

    prod.join();
    cons.join();

    auto end_time = stopwatch::now();

    Print("Done in {0} ms", std::chrono::duration_cast<std::chrono::milliseconds>(end_time - begin_time).count());
}
