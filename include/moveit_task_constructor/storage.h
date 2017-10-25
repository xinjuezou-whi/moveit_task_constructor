// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/macros/class_forward.h>

#include <list>
#include <vector>
#include <deque>
#include <cassert>
#include <functional>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit { namespace task_constructor {

class SolutionBase;
MOVEIT_CLASS_FORWARD(InterfaceState)
MOVEIT_CLASS_FORWARD(Interface)
typedef std::weak_ptr<Interface> InterfaceWeakPtr;
MOVEIT_CLASS_FORWARD(Stage)


/** singleton repository for pointers of T
 *
 *  Serves as an efficient mapping from IDs to the associated T instance.
 *  size() serves as the new ID after insertion of a new item,
 *  i.e. we have item IDs from 1 onwards. ID == 0 is invalid.
 */
template <typename T>
class Repository : private std::deque<const T*> {
	typedef std::deque<const T*> base_type;

	Repository() {}
	Repository(const Repository&) = delete;
	void operator=(const Repository&) = delete;

public:
	static Repository& instance() {
		static Repository instance_;
		return instance_;
	}

	size_t add(const T* item) {
		this->push_back(item);
		return this->size();
	}

	void remove(const T* item) {
		// invalidate entry
		this->base_type::at(item->id()-1) = nullptr;
	}

	const T& operator[](size_t id) const {
		return *this->base_type::at(id-1);
	}
};


/** InterfaceState describes a potential start or goal state for a planning stage.
 *
 *  A start or goal state for planning is essentially defined by the state of a planning scene.
 */
class InterfaceState {
	friend class SolutionBase; // addIncoming() / addOutgoing() should be called only by SolutionBase
public:
	/// get ID of this state
	size_t id() const { return id_; }
	// TODO turn this into priority queue
	typedef std::deque<SolutionBase*> Solutions;

	/// create an InterfaceState from a planning scene
	InterfaceState(const planning_scene::PlanningSceneConstPtr& ps);

	/// copy an existing InterfaceState, but not including incoming/outgoing trajectories
	InterfaceState(const InterfaceState& existing);

	~InterfaceState();

	inline const planning_scene::PlanningSceneConstPtr& scene() const { return scene_; }
	inline const Solutions& incomingTrajectories() const { return incoming_trajectories_; }
	inline const Solutions& outgoingTrajectories() const { return outgoing_trajectories_; }

private:
	// these methods should be only called by SolutionBase::set[Start|End]State()
	inline void addIncoming(SolutionBase* t) { incoming_trajectories_.push_back(t); }
	inline void addOutgoing(SolutionBase* t) { outgoing_trajectories_.push_back(t); }

private:
	inline void registerID();

	size_t id_;
	planning_scene::PlanningSceneConstPtr scene_;
	// TODO: add PropertyMap: std::map<std::string, std::any> to allow passing of parameters or attributes
	// TODO: add visualization_msgs::MarkerArray to allow for any complex visualization of the state
	Solutions incoming_trajectories_;
	Solutions outgoing_trajectories_;
};


/** Interface provides a list of InterfaceStates available as input for a stage.
 *
 *  This is essentially an adaptor to a container class, to allow for notification
 *  of the interface's owner when new states become available
 */
class Interface : protected std::list<InterfaceState> {
public:
	typedef std::list<InterfaceState> container_type;
	typedef std::function<void(const container_type::iterator&)> NotifyFunction;
	Interface(const NotifyFunction &notify);

	// add a new InterfaceState, connect the trajectory (either incoming or outgoing) to the newly created state
	// and finally run the notify callback
	container_type::iterator add(InterfaceState &&state, SolutionBase* incoming, SolutionBase* outgoing);

	// clone an existing InterfaceState, but without its incoming/outgoing connections
	container_type::iterator clone(const InterfaceState &state);

	using container_type::value_type;
	using container_type::reference;
	using container_type::const_reference;

	using container_type::iterator;
	using container_type::const_iterator;
	using container_type::reverse_iterator;
	using container_type::const_reverse_iterator;

	using container_type::empty;
	using container_type::size;
	using container_type::clear;
	using container_type::front;
	using container_type::back;

	using container_type::begin;
	using container_type::cbegin;
	using container_type::end;
	using container_type::cend;
	using container_type::rbegin;
	using container_type::crbegin;
	using container_type::rend;
	using container_type::crend;

private:
	const NotifyFunction notify_;
};


class StagePrivate;
class SubTrajectory;

/// SolutionTrajectory is composed of a series of SubTrajectories
typedef std::vector<const SubTrajectory*> SolutionTrajectory;

class SolutionBase {
public:
	// retrieve ID of this solution in repository
	inline size_t id() const { return id_; }
	// TODO: get rid of creator (only used in SerialContainer)
	inline const StagePrivate* creator() const { return creator_; }
	inline double cost() const { return cost_; }

	inline const InterfaceState* start() const { return start_; }
	inline const InterfaceState* end() const { return end_; }

	inline void setStartState(const InterfaceState& state){
		assert(start_ == NULL);
		start_ = &state;
		const_cast<InterfaceState&>(state).addOutgoing(this);
	}

	inline void setEndState(const InterfaceState& state){
		assert(end_ == NULL);
		end_ = &state;
		const_cast<InterfaceState&>(state).addIncoming(this);
	}
	void setCost(double cost);

	/// flatten this solution to full solution trajectory, appending to solution
	virtual void flattenTo(SolutionTrajectory& solution) const = 0;

protected:
	SolutionBase(StagePrivate* creator, double cost)
	   : creator_(creator), cost_(cost)
	{
		id_ = Repository<SolutionBase>::instance().add(this);
	}
	~SolutionBase() {
		Repository<SolutionBase>::instance().remove(this);
		id_ = 0; // invalidate ID
	}

private:
	// unique ID of this solution
	size_t id_;
	// back-pointer to creating stage, allows to access sub-solutions
	StagePrivate *creator_;
	// associated cost
	double cost_;

	// begin and end InterfaceState of this solution/trajectory
	const InterfaceState* start_ = nullptr;
	const InterfaceState* end_ = nullptr;
};


// SubTrajectory connects interface states of ComputeStages
class SubTrajectory : public SolutionBase {
public:
	explicit SubTrajectory(StagePrivate* creator, const robot_trajectory::RobotTrajectoryPtr& traj, double cost);

	robot_trajectory::RobotTrajectoryConstPtr trajectory() const { return trajectory_; }
	const std::string& name() const { return name_; }
	void setName(const std::string& name) { name_ = name; }

	void flattenTo(SolutionTrajectory& solution) const override {
		solution.push_back(this);
	}

private:
	const robot_trajectory::RobotTrajectoryPtr trajectory_;
	// trajectories could have a name, e.g. a generator could name its solutions
	std::string name_;
};


enum TraverseDirection { FORWARD, BACKWARD };
template <TraverseDirection dir>
const InterfaceState::Solutions& trajectories(const SolutionBase &start);

template <> inline
const InterfaceState::Solutions& trajectories<FORWARD>(const SolutionBase &solution) {
	return solution.end()->outgoingTrajectories();
}
template <> inline
const InterfaceState::Solutions& trajectories<BACKWARD>(const SolutionBase &solution) {
	return solution.start()->incomingTrajectories();
}

} }
