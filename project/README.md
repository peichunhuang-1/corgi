README
===
說明
---
本專案分為兩個部分：
- 專案核心
- 專案功能區塊

專案核心的部分主要包含專案功能之間的通訊以及資料記錄，專案功能區塊為實現機器人特定功能的部分，而功能區塊之間的通訊建立在核心上。
核心區的代碼基本上不需要去看，他幾乎都是寫好後已經編過方便各位使用的功能，他與ROS的架構類似，若是熟悉ROS的人可以略過此部分說明。

- NodeCore： 如果要在各個功能節點(node)間通訊的話，就必須執行這一個執行檔，他的功能是註冊節點收發的話題（topics），並且綁定收發相同話題的節點。

- NodeHandler：這是在專案功能中，需要各位在自己撰寫的節點中宣告的物件，基於這一個物件，各位可以宣告發布者（publisher）與訂閱者（subscriber），而通過發布與訂閱，各位可以拿到需要的資料，或者與其他的節點分享自己運算後的資料。

概念上大致如此，後續為目前各檔案說明與核心使用的說明範例。

專案核心
---
#### 說明
核心的部分源碼是可以不必去看，可以直接下載編譯後安裝，但需要預先安裝一些c++的函式庫，基本上是都要安裝，不過如果是要跑在本機端不需要讀取真實的imu資料，可以忽略掉imu的部分，但需要註解掉所有用到該函式庫的部分。
:::info
1. nlopt : 非線性最佳化函式庫
2. boost_1_73_0 : c++準標準庫
3. cereal : 序列化函式庫
4. eigen-3.4.0 : 線性矩陣運算庫
5. mip-sdk : imu通訊庫
6. recommend cmake version 3.10.0 or above
:::

:::spoiler Build nlopt

    $ git clone https://github.com/stevengj/nlopt.git
    $ cd nlopt
    $ mkdir build
    $ cd build
    $ cmake .. -DNLOPT_PYTHON=OFF -DNLOPT_MATLAB=OFF
    $ make
    $ sudo make install
:::

:::spoiler Build cereal
    $ git clone https://github.com/USCiLab/cereal.git
    $ cd cereal
    $ mkdir build
    $ cd build
    $ cmake .. -DJUST_INSTALL_CEREAL = ON
    $ make 
    $ sudo make install
:::

:::spoiler Build mip-sdk

    $ git clone https://github.com/LORD-MicroStrain/mip_sdk.git
    $ cd mip-sdk
    $ mkdir build
    $ cd build
    $ cmake .. -DWITH_SERIAL=1
    $ make 
    $ sudo make install
:::

:::spoiler Build eigen

    Download from https://gitlab.com/libeigen/eigen/-/releases/3.4.0
    $ cd eigen-3.4.0
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
:::

:::spoiler Build boost

    Download from https://www.boost.org/users/history/version_1_73_0.html
    $ cd boost_1_73_0
    $ ./bootstrap.sh --prefix=/usr/local
    $ ./b2 install

:::


#### 編譯與安裝

安裝好前述的函式庫後，即可以下載core並安裝。

    $ cd core
    $ rm -r build
    $ mkdir build && cd build
    $ sudo make install
這些指令會把core編譯後安裝執行檔至你的系統執行區和函式庫的預設區域，在執行core的指令時就可以直接在終端裡打該執行命令的名稱，引用時也可以直接向其他的c++函式庫一樣引用。

#### 核心功能

核心主要的功能是用以通訊、記錄數據、檢查數據以及參數化一些必要的參數。
- 通訊：通訊的架構是基於節點（node）與話題（topic），每一個執行檔都是一個節點，而節點可以從話題收發資料，不同節點之間就是透過話題交流資料。
- 紀錄：可以透過撰寫好的NodeFile指令紀錄特定話題的資訊，使用方法如下：
    ```bash
    $ NodeFile 
    -t ${topic_name1} ${topic_name2} 
    -f ${frquency} 
    -m ${master_ip} 
    -p ${master_port} 
    -l ${local_ip}
    -n ${filename}
    ```
    其中的參數是由使用者決定的，Master的資料是指執行NodeCore的裝置IP和port，Local是指現在要記錄數據的裝置IP，還可以指定紀錄的頻率和話題的名稱，話題不拘限數量，最後是紀錄的路徑與檔名，我建議是使用絕對路徑。
- 檢驗：檢驗的部分目前還只能輸出文字的形式，在終端機輸出話題的資料，使用方法如下：
    ```bash
    $ NodeEcho 
    -t ${topic_name}
    -f ${frquency} 
    -m ${master_ip} 
    -p ${master_port} 
    -l ${local_ip}
    ```
    參數如上，其實也不拘限話題的數量，但如果是要除錯通常是只會看一個。

- 參數化：上述的參數就是基於這個參數化的功能，若你的節點也需要這些參數需要include相對應的抬頭檔。


專案功能
---
#### 檔案介紹

:::spoiler corgi

>:::spoiler project
>>:::spoiler mathematic
>>>:::spoiler coordinate_transform.hpp
>>>此標頭檔提供以最佳化來進行座標轉換的函式。
>>>:::
>>>:::spoiler coordinate.hpp
>>>此標頭檔提供卡氏與極座標的轉換與基礎操作。
>>>:::
>>>:::spoiler leg_model.hpp
>>>此標頭檔提供足部的模型計算，包含順逆運動學等。
>>>:::
>>>:::spoiler quaternion.hpp
>>>此標頭檔提四元數的計算。
>>>:::
>>>:::spoiler rigid_body.hpp
>>>此標頭檔提供剛體的基礎運算。
>>:::
>>:::spoiler imu
>>>:::spoiler imu.cpp
>>>編譯後為imu的讀取節點，發送imu資料到其他節點。
>>>:::
>>:::
>>:::spoiler odometry
>>>:::spoiler odometry.cpp
>>>編譯後讀取imu、腳步的資料，計算位置與姿態，發送給其他節點。
>>>:::
>>:::
>>:::spoiler legs
>>>:::spoiler legs.cpp
>>>編譯後讀取theta、beta角，並計算出腳尖位置、觸地判斷等發送給其他節點。
>>>:::
>>:::
>>:::spoiler force
>>>:::spoiler force_optimizer.cpp
>>>編譯後讀取位置、姿態、腳尖位置等並結合剛體運動計算出最佳的控制力，發送給其他節點。
>>>:::
>>:::
>>:::spoiler motors
>>>:::spoiler motors.cpp
>>>編譯後讀取原始的encoder資料，彙整成個別腳的theta、beta角，並發送給其他節點，同時讀取馬達控制的角度。
>>>:::
>>:::
>>:::spoiler cpg
>>>:::spoiler cpg.cpp
>>>編譯後計算步態相位，發送給其他節點。
>>>:::
>>:::
>>:::spoiler foothold
>>>:::spoiler foothold.cpp
>>>編譯後讀取cpg、以及odometry的資料，並計算馬達的控制角度，發送給馬達節點。
>>>:::
>>:::
>>CMakeLists.txt


>:::spoiler build
>編譯用資料夾


>:::spoiler devel
>二進檔案存放區


>CMakeLists.txt
:::
#### 編譯與新增檔案


1. 在project中新增一個資料夾，假設為一個測試功能的資料夾test。
2. 在test中新增cpp、hpp檔案，撰寫應用功能。
3. 在test中新增CMakeLists.txt，可以參考其他的資料夾中的檔案。
    ```
    add_executable(test test.cpp test.hpp)
    target_link_libraries(test ${necessary-libraries})
4. 在project中的CMakeLists.txt中新增test的資料夾。
    ```
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test)
5. 打開terminal，並到目錄build下: 
    ```
    $ cmake .. && make
6. 至此執行檔test會放到devel目錄下。

#### 執行檔案

1. 若要單一的執行某一個檔案，而需要用到專案核心的通訊功能，則需要執行專案的核心，若按照前述的流程，執行檔應該已經被安裝到系統執行路徑中，可以直接在終端中執行。
    ```
    $ NodeCore
2. 若要執行整個專案，則方便一點的方法是寫一個腳本檔(.sh)，範例如下，需注意參數化的輸入是需要在c++檔案中編寫的，可以參考imu.cpp。
    ``` bash
    # /bin/bash
    trap "kill -- -$$" INT # add so ctrl c can kill all process
    echo Master IP:        # print user guide to input master ip
    read master_ip         # read master ip
    echo Master port:      # print user guide to input master port
    read master_port       # read master port
    echo Local IP:         # print user guide to input local ip
    read local_ip          # read local ip
    echo path:             # print user guide to input path of executable files
    read path              # read path
    NodeCore -m "$master_ip" -l "$local_ip" -p "$master_port" & 
    sleep .1   

    ./"$path"/cpg -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
    sleep .1

    ./"$path"/odometry -m "$master_ip" -l "$local_ip" -p "$master_port" -f 500 & 
    wait
    ```