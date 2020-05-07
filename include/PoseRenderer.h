//
// Created by tomlucas on 23.03.20.
//

#ifndef CYLINDEREXAMPLE_POSERENDERER_H
#define CYLINDEREXAMPLE_POSERENDERER_H
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <vtkAnimationScene.h>
#include <vtkAnimationCue.h>

#include <list>
#include <memory>
#include <chrono>
#include <thread>
#include <Eigen/Geometry>

namespace adekf::viz {
    /**
     * Baseclass to read the estimated position and orientation from an estimator.
     *
     * Implement a subclass of this to display the pose of an estimator
     */
    class PoseReader{
    public:
        /**
         * Returns the estimated position
         * @return The estimated position
         */
        virtual Eigen::Vector3d getPosition() =0;
        /**
         * Returns the estimated orientation
         * @return The estimated orientation.
         */
        virtual Eigen::Quaterniond getOrientation()=0;
        virtual ~PoseReader(){};
    };


    /**
     * This is a generic implementation to read the estimated pose.
     *
     * It works under the following Conditions:
     * 1. The estimator has the public field mu (estimated state)
     * 2. The state has the field position which is an 3x1 Eigen::Vector
     * 3. The state has the field orientation which is an Eigen::Quaternion
     *
     * In short, call your position "position" and your orientation "orientation"
     *
     */
    template<class EstimatorType>
    class GenericPoseReader : public PoseReader{
        EstimatorType * estimator;
    public:
        GenericPoseReader(EstimatorType * estimator):estimator(estimator){

        }
        virtual ~GenericPoseReader(){};

        Eigen::Vector3d getPosition() override {
            return estimator->mu.position;
        }

        Eigen::Quaterniond getOrientation() override {
            return estimator->mu.orientation;
        }
    };

    /**
   * This is a generic implementation to read the estimated position (not orientation).
   *
   * It works under the following Conditions:
   * 1. The estimator has the public field mu (estimated state)
   * 2. The state has the field position which is an 3x1 Eigen::Vector
   *
   * In short, call your position "position"
   *
   */
    template<class EstimatorType>
    class GenericPositionReader : public PoseReader{
        EstimatorType * estimator;
    public:
        GenericPositionReader(EstimatorType * estimator):estimator(estimator){

        }
        virtual ~GenericPositionReader(){}


        Eigen::Vector3d getPosition() override {
            return estimator->mu.position;
        }

        Eigen::Quaterniond getOrientation() override {
            return Eigen::Quaterniond::Identity();
        }
    };

    /**
     * This class implements Methods to display the estimated Pose of an object with little effort.
     *
     * All that is required to display the pose is:
     * 1.Call @see adekf::viz::initGuis() (recommend) or
     * call @see initGui()
     * 2. Add various estimators to display by calling
     *  @see displayPose(<estimator_ptr>, <color_name>) or
     *  @see displayPosition(<estimator_ptr>, <color_name>
     *
     *  Note: These commands work for the adekf with a state build with ADEKF_MANIFOLD where:
     *  the state has a 3-DOF vector called "position" (required for both commands) and
     *  an adekf::SO3 Manifold called "orientation"  (required for displayPose)
     *  If you require something custom implement a subclass of PoseReader and use the method
     *  displayPoseGeneric
     *
     *  3. Call @see adekf::viz::runGuis() (recommend) or
     *  repeatedly  call @see updateWindow() from the main thread
     *
     *  4. call @see adekf::viz::finishGuis() (recommend) or
     *  call @see disposeWindow()
     *
     *
     *  The Camera can be configured by calling @see setCameraPosition and @see setFocalPoint.
     *  Autotracking of an estimator can be enabled by calling @see setTrackedEstimator
     */
    class PoseRenderer {
        //the windowInteractor that contains all graphic objects
        inline static vtkSmartPointer<vtkRenderWindowInteractor> window;
        //The renderer of the window
        inline static vtkSmartPointer<vtkRenderer> renderer;
        //a list with all PoseReaders (stores the estimators) and the corresponding actors in the graphics
        inline static std::list<std::pair<PoseReader *,vtkSmartPointer<vtkActor>  >> posesAndActors;
        //The focus point of the automatic tracking
        inline static std::shared_ptr<PoseReader> focus;
        //The view vector of the automatic tracking
        inline static Eigen::Vector3d view_vector;

        /**
         * Internally used to create a box actor of the given color.
         *
         * The box is added to the renderer and will be displayed from now on.
         * @param color the color of the box
         * @param scale The box size is scalled by this value base size [2 1 0.5]
         * @return The created box
         */
        static  vtkSmartPointer<vtkActor> createActor(const char* color,double scale=1.0);
    public:
        /**
         * Indicates whether the window is still open
         * @return True if the window is closed (after pressing q,e or the Exit Button), false otherwise
         */
        static bool isDone();

        /**
         * Set the position of the camera of the 3d Window
         * @param x  The x-coordinate
         * @param y  The y-coordinate
         * @param z  The z-coordinate
         */
        static void setCameraPosition(int x, int y,int z);


        /**
         * Set the focal point  (where to look ) of the camera of the 3d Window.
         *
         * Should be called after @see setCameraPosition.
         * @param x  The x-coordinate
         * @param y  The y-coordinate
         * @param z  The z-coordinate
         */
        static void setFocalPoint(int x,int y, int z);


        /**
         * Sets the estimator to be tracked by the camera and enables auto tracking.
         *
         * After the call the camera will hold the estimated pose in the center of the view.
         * The camera will stay at estimated position + view_vector
         * @tparam EstimatorType The type of the estimator
         * @param estimator  The pointer to the estimator to be tracked. Does not take ownership
         * @param view_vector The offset vector of the camera to the tracked position
         */
        template<class EstimatorType>
        static void setTrackedEstimator(EstimatorType * estimator,const Eigen::Vector3d &view_vector){
            focus=std::make_shared<GenericPositionReader<EstimatorType>>(estimator);
            PoseRenderer::view_vector=view_vector;
        }




        /**
         * Initializes the graphical interface for 3D pose rendering
         *
         * Has to be called before any call to displayPose or displayPosition
         * Sets the camera position to [0 0 80] and the focal point to [0 0 0]
         */
        static void initGui() ;

        /**
         * Updates all registered actors to show the poses.
         *
         * Also handles Events as Close Window or camera manipulations
         */
        static void updateWindow();


        /**
         * Destroys objects and frees allocated memory.
         *
         * Does not affect estimator pointers
         */
         static void disposeWindow();

         /**
          * Display the estimated position of the given estimator as a box.
          *
          * Requires that the estimator has a field mu (estimated state) with a field position (estimated position)
          * Stores the pointer  (no ownership) to the estimator and reads the position with a @see GenericPositionReader
          * Call @see initGuis() before
          * @tparam EstimatorType The type of the estimator
          * @param estimator The pointer to the estimator
          * @param color The color of the displayed box
          */
        template<class EstimatorType>
        static void displayPosition(EstimatorType * estimator, const char * color){
            displayPoseGeneric<EstimatorType,GenericPositionReader >(estimator,color);
        }

        /**
          * Display the estimated pose of the given estimator as a box.
          *
          * Requires that the estimator has a field mu (estimated state) with a field position (estimated position) and  a field orientation (estimated orientation)
          * Stores the pointer (no ownership) to the estimator and reads the position with a @see GenericPoseReader
          * Call @see initGuis() before
          * @tparam EstimatorType The type of the estimator
          * @param estimator The pointer to the estimator
          * @param color The color of the displayed box
          */
        template<class EstimatorType>
        static void displayPose(EstimatorType * estimator, const char * color){
            displayPoseGeneric<EstimatorType,GenericPoseReader >(estimator,color);
        }

        /**
         * Display the estimated pose of the given estimator as a box
         *
         * Generic implementation to display poses. It can be used if @see displayPose and @see displayPosition are unsuitable.
         * Stores the pointer (no ownership) to the estimator and reads the position with a custom PoseReader
         * Call @see initGuis() before
         * @tparam EstimatorType The type of the estimator
         * @tparam ReaderType The custom PoseReader type which implements how to read the pose from the given estimator
         * @param estimator The pointer to the estimator
         * @param color  The color of the displayed box
         */
        template<class EstimatorType, template <class> class ReaderType>
        static void displayPoseGeneric(EstimatorType *estimator, const char * color) {
            posesAndActors.push_back(std::make_pair<PoseReader *, vtkSmartPointer<vtkActor> >(new ReaderType<EstimatorType>(estimator),createActor(color)));
        }




    };

}


#endif //CYLINDEREXAMPLE_POSERENDERER_H
