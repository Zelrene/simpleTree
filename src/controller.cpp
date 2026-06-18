/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Jan Hackenberg, University of Freiburg.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/
#include "controller.h"

Controller::Controller ()
{}

Controller::~Controller ()
{}

pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr
Controller::getCurvaturePtr ()
{
    return curvature_ptr;
}

void
Controller::init (int argc,
                  char *argv[])
{
    // Below is original code
    QApplication a (argc, argv);
    QRect screenres = QApplication::desktop ()->screenGeometry (1/*screenNumber*/);

    if (argc == 3)
    {
        std::string input_dir = argv[1];
        std::string output_dir = argv[2];

        // run after event loop starts
        QTimer::singleShot(
            0,
            [=]()
            {
                this->runBatch(
                    input_dir,
                    output_dir
                );

                // auto close app
                QApplication::quit();
            }
        );

        a.exec();
        return;
    }


    this->gui_ptr.reset (new PCLViewer);
    this->gui_ptr->move (QPoint (screenres.x (), screenres.y ()));
    this->gui_ptr->resize (screenres.width (), screenres.height ());
    this->gui_ptr->init();



    this->gui_ptr->show ();
    this->gui_ptr->connectToController (shared_from_this ());
    a.exec ();
}

void Controller::runCLI(std::string input, std::string output)
{
    headless = true;
    ImportPCD import(input, shared_from_this());

    cloud_ptr = import.getCloud();
    std::cout << "[DEBUG] after import: cloud = " 
          << (cloud_ptr ? cloud_ptr->size() : -1) << std::endl;

    auto cloud = getCloudPtr();
    if (!cloud) return;

    EigenValueEstimator( cloud, e1, e2, e3, isStem, 0.035f );
    std::cout << "[DEBUG] after eigen: "
          << "e1=" << e1.size()
          << " e2=" << e2.size()
          << " e3=" << e3.size()
          << " isStem=" << isStem.size()
          << std::endl;

    StemPointDetection detect(cloud, isStem);
    setIsStem(detect.getStemPtsNew());
    std::cout << "[DEBUG] after stem detection: isStem="
          << getIsStem().size()
          << std::endl;

    Method_Coefficients mc{};
    mc.sphere_radius_multiplier = 1.8f;
    mc.epsilon_cluster_stem = 0.02f;
    mc.epsilon_cluster_branch = 0.008f;
    mc.epsilon_sphere = 0.02f;
    mc.minPts_ransac_stem = 2000;
    mc.minPts_cluster_stem = 12;
    mc.min_radius_sphere_stem = 0.035f;
    mc.min_radius_sphere_branch = 0.025f;
    mc.bin_width = 1.0f;
    mc.max_iterations = 2;

    float min_height = -1.0f;
    float bin_width = mc.bin_width;

    std::cout << "[DEBUG] BEFORE SphereFollowing" << std::endl;
    std::cout << "[DEBUG] cloud size: " << cloud->size() << std::endl;
    std::cout << "[DEBUG] stem size: " << getIsStem().size() << std::endl;
    SphereFollowing sf( cloud, getIsStem(), 1, mc, min_height, bin_width );
    std::cout << "[DEBUG] AFTER SphereFollowing cylinders="
          << sf.getCylinders().size()
          << std::endl;


    std::cout << "[DEBUG] BEFORE Tree construction" << std::endl;

    auto cyl = sf.getCylinders();
    std::cout << "[DEBUG] cylinders from SF = " << cyl.size() << std::endl;

    if (cyl.empty())
    {
        std::cout << "[BATCH] EMPTY CYLINDERS -> skipping file" << std::endl;
        return;   // OR continue batch loop
    }

    for (const auto& c : cyl)
    {
        if (c.values.size() < 6 || !std::isfinite(c.values[0]))
            throw std::runtime_error("Tree: invalid cylinder detected");
    }
    
    std::cout << "[DEBUG] cloud ptr: " << (cloud ? "OK" : "NULL") << std::endl;
    std::cout << "[DEBUG] cloud size: " << cloud->size() << std::endl;
    std::cout << "[DEBUG] output: " << output << std::endl;

    std::cout << "[DEBUG] constructing Tree..." << std::endl;
    auto tree = boost::make_shared<simpleTree::Tree>(
        cyl,
        cloud,
        output,
        true
    );
    std::cout << "[DEBUG] Tree constructed" << std::endl;

    setTreePtr(tree);

    fs::create_directories(output);
    ExportPly(tree->getCylinders(), output, "tree");
    WriteCSV(tree, output);
}

void Controller::runBatch(std::string input_dir, std::string output_root)
{
    std::vector<std::string> inputs;

for (const auto& entry : fs::directory_iterator(input_dir))
{
    if (!fs::is_regular_file(entry.path())) continue;

    if (entry.path().extension() == ".pcd")
        inputs.push_back(entry.path().string());
}

std::sort(inputs.begin(), inputs.end());

if (inputs.empty())
{
    std::cout << "[BATCH] No .pcd files found in input directory." << std::endl;
    return;
}

fs::create_directories(output_root);

for (const auto& in : inputs)
{
    try
    {
        std::string stem = fs::path(in).stem().string();
        std::string out_dir = output_root + "/" + stem;

        fs::create_directories(out_dir);

        std::cout << "\n[BATCH] Processing: " << in << std::endl;

        runCLI(in, out_dir);

        std::cout << "[BATCH] Done: " << in << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << "[BATCH] ERROR file: " << in
                  << " | " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "[BATCH] UNKNOWN ERROR file: " << in << std::endl;
    }
}
}

bool Controller::hasGui() const {
    return !headless && gui_ptr;
}

PCLViewer* Controller::GUI() {
    return hasGui() ? gui_ptr.get() : nullptr;
}

void Controller::safeProcessEvents() {
    if (!headless)
        QCoreApplication::processEvents();
}

void Controller::safeWriteConsole(const QString& msg) {
    if (gui_ptr)
        gui_ptr->writeConsole(msg);
}

void Controller::safeUpdateProgress(int v) {
    if (gui_ptr)
        gui_ptr->updateProgress(v);
}


std::string
Controller::getTreeID ()
{
    return treeID;
}

void
Controller::setTreeID (std::string treeID)
{
    this->treeID = treeID;
}

void
Controller::setTreeID (QString treeID)
{
    this->treeID = treeID.toStdString();
}

PointCloudI::Ptr
Controller::getCloudPtr ()
{
    return this->cloud_ptr;
}

void
Controller::setCloudPtr (pcl::PointCloud<PointI>::Ptr cloud_ptr, bool changeView )
{
    this->cloud_ptr = cloud_ptr;
    this->cloud_ptr->width = cloud_ptr->points.size();
    this->cloud_ptr->height = 1;
    this->tree_ptr = 0;
    this->e1.clear();
    this->e2.clear();
    this->e3.clear();
    this->isStem.clear();
    this->curvature_ptr.reset(new CurvatureCloud);
    if(changeView)
    {
        this->gui_ptr->setCloudPtr (cloud_ptr, false );
    } else {
        this->gui_ptr->setCloudPtr (cloud_ptr, false );
    }
}

void
Controller::setCloudPtr (PointCloudI::Ptr cloud_ptr,
                         CurvatureCloud::Ptr curvature_ptr)
{
    this->cloud_ptr = cloud_ptr;
    this->cloud_ptr->width = cloud_ptr->points.size();
    this->cloud_ptr->height = 1;
    this->curvature_ptr = curvature_ptr;
    this->tree_ptr = 0;
    this->e1.clear();
    this->e2.clear();
    this->e3.clear();
    this->isStem.clear();
    this->gui_ptr->setCloudPtr (cloud_ptr);
}


void
Controller::setCurvaturePtr (CurvatureCloud::Ptr curvature_ptr)
{
    this->curvature_ptr = curvature_ptr;
    this->gui_ptr->setCloudPtr (cloud_ptr);
}

boost::shared_ptr<PCLViewer>
Controller::getGuiPtr ()
{
    if (headless) return nullptr;
    return this->gui_ptr;
}

boost::shared_ptr<simpleTree::Tree>
Controller::getTreePtr ()
{
    return this->tree_ptr;
}

void
Controller::setTreePtr (boost::shared_ptr<simpleTree::Tree> tree_ptr)
{
    this->tree_ptr = tree_ptr;
    this->gui_ptr->setCloudPtr(cloud_ptr);
    this->gui_ptr->setTreePtr (tree_ptr);
}

void
Controller::setE1 ( std::vector<float> e1)
{
    this->e1 = e1;
}

void
Controller::setE2 ( std::vector<float> e2)
{
    this->e2 = e2;
}

void
Controller::setE3 ( std::vector<float> e3)
{
    this->e3 = e3;
}

void
Controller::setIsStem(const std::vector<bool>& isStem) {
    this->isStem = isStem;
}
