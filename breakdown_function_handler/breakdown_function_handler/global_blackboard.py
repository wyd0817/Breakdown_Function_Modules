import networkx as nx
import os
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import json
import asyncio

class Blackboard:
    def __init__(self):
        self.graph = nx.DiGraph()
        self.last_visualized_state = None
        self.folder_name = None
        self.lock = asyncio.Lock()
        self.initialize_folders()

    def initialize_folders(self):
        current_time = time.strftime("%Y%m%d-%H%M%S")
        self.folder_name = os.path.join(os.path.dirname(os.path.dirname(__file__)), "..", "..", "blackboard_logs", current_time)
        os.makedirs(self.folder_name, exist_ok=True)

    async def save_state(self):
        current_time = time.strftime("%Y%m%d-%H%M%S")
        state_file = os.path.join(self.folder_name, f"{current_time}.json")
                
        # Check if there are any nodes in the graph
        if not self.graph.nodes:
            state = {}
            print("\033[92mAll tasks on the blackboard have been completed.\033[0m")
        else:
            state = {
                node: {
                    'completed': data.get('completed', False),
                    'task_info': data.get('task_info', {})
                }
                for node, data in self.graph.nodes(data=True)
            }

        async with self.lock:
            with open(state_file, 'w') as f:
                json.dump(state, f, indent=4, separators=(',', ': '), default=str)
                f.write('\n')

    async def load_state(self):
        latest_state_file = self.get_latest_state_file()
        if latest_state_file:
            async with self.lock:
                with open(latest_state_file, 'r') as f:
                    state = json.load(f)
            self.graph = nx.DiGraph()
            for task_id, data in state.items():
                self.graph.add_node(task_id, **data)
            # Rebuild edges
            for task_id in self.graph.nodes:
                task_info = self.graph.nodes[task_id]['task_info']
                for dep_id in task_info.get('dependencies', []):
                    if self.graph.has_node(dep_id):
                        self.graph.add_edge(dep_id, task_id)

    def get_latest_state_file(self):
        files = [f for f in os.listdir(self.folder_name) if f.endswith('.json')]
        if not files:
            return None
        latest_file = max(files, key=lambda f: os.path.getctime(os.path.join(self.folder_name, f)))
        return os.path.join(self.folder_name, latest_file)

    async def add_task(self, task_id, task_info):
        dependencies = task_info.get('dependencies', [])
        self.graph.add_node(task_id, completed=False, task_info=task_info)
        if dependencies:
            for dep_id in dependencies:
                self.graph.add_edge(dep_id, task_id)
        print(f"\033[92mTask added: {task_id}, Dependencies: {dependencies}\033[0m")
        await self.save_state()
        await self.visualize_if_changed()


    async def mark_task_completed(self, task_id):
        if self.graph.has_node(task_id) and not self.graph.nodes[task_id].get('completed', False):
            self.graph.nodes[task_id]['completed'] = True
            print(f'Marked task as completed: {task_id}')
            await self.save_state()
            await self.visualize_if_changed()

    async def delete_task(self, task_id):
        async with self.lock:
            if self.graph.has_node(task_id):
                self.graph.remove_node(task_id)
                print(f"Task {task_id} removed from graph")
                await self.save_state()
                await self.visualize_if_changed()

    async def get_ready_tasks(self):
        ready_tasks = []
        for task_id, node_data in self.graph.nodes(data=True):
            if not node_data.get('completed', False) and not node_data.get('in_progress', False):
                predecessors = list(self.graph.predecessors(task_id))
                all_deps_completed = True
                for dep_id in predecessors:
                    if not self.graph.nodes[dep_id].get('completed', False):
                        all_deps_completed = False
                        break
                if all_deps_completed:
                    task_info = node_data.get('task_info', {})
                    if task_info:
                        ready_tasks.append(task_info)
        return ready_tasks

    async def check_dependencies(self, task_info):
        task_id = task_info.get('task_id')
        dependencies = task_info.get('dependencies', [])
        print(f"\033[95mChecking dependencies for task {task_id}: {dependencies}\033[0m")
        
        all_deps_completed = True
        for dep_id in dependencies:
            is_completed = await self.is_task_completed(dep_id)
            print(f"\033[93mDependency {dep_id} completed: {is_completed}\033[0m")
            
            if not is_completed:
                all_deps_completed = False
                print(f"\033[91mTask {task_id} is waiting for dependency {dep_id} to complete.\033[0m")
                break
        
        if all_deps_completed:
            print(f"\033[92mAll dependencies for task {task_id} are completed.\033[0m")
        else:
            print(f"\033[91mNot all dependencies for task {task_id} are completed yet.\033[0m")
            
        return all_deps_completed

    async def is_task_completed(self, task_id):
        if self.graph.has_node(task_id):
            return self.graph.nodes[task_id].get('completed', False)
        return False
    
    async def get_task_info(self, task_id):
        if self.graph.has_node(task_id):
            return self.graph.nodes[task_id].get('task_info', None)
        return None

    async def visualize_if_changed(self):
        current_state = await self.get_graph_state()
        if current_state != self.last_visualized_state:
            await self.visualize()
            self.last_visualized_state = current_state

    async def get_graph_state(self):
        await self.load_state()  # Get the latest state of the graph
        return frozenset((node, data.get('completed', False)) for node, data in self.graph.nodes(data=True))

    async def add_dependency(self, task_id, dependency_id):
        async with self.lock:
            if self.graph.has_node(task_id) and self.graph.has_node(dependency_id):
                self.graph.add_edge(dependency_id, task_id)
                print(f"Dependency added: {task_id} depends on {dependency_id}")
                await self.save_state()
                await self.visualize_if_changed()
            else:
                print(f"Warning: Could not add dependency. Task {task_id} or {dependency_id} not found.")

    async def visualize(self):
        current_time = time.strftime("%Y%m%d-%H%M%S")
        file_path = os.path.join(self.folder_name, f"{current_time}.png")

        if len(self.graph.nodes) == 0:
            plt.figure()
            plt.savefig(file_path, bbox_inches='tight')
            plt.close()
            print("\033[92mAll tasks on the blackboard have been completed.\033[0m")
            return
        
        # Custom layout: arrange nodes in a horizontal line
        pos = {}
        width = len(self.graph.nodes)
        for i, node in enumerate(self.graph.nodes()):
            pos[node] = (i, 0)
        
        labels = {node: self.graph.nodes[node]['task_info'].get('task_name', node) for node in self.graph.nodes()}
        
        plt.figure(figsize=(24, 16))
        nx.draw(self.graph, pos, with_labels=True, labels=labels, node_size=3000*6, node_color="lightblue", 
                font_size=10, font_weight="bold", edge_color="gray", linewidths=2, edgecolors="black", arrows=True, arrowsize=100, width=2)
        
        plt.savefig(file_path, bbox_inches='tight')
        plt.close()
        print("\033[92mGraph saved to {}\n\033[0m".format(file_path))

    async def remove_task(self, task_id):
        async with self.lock:
            if self.graph.has_node(task_id):
                self.graph.remove_node(task_id)
                print(f"Task {task_id} removed from graph")
                await self.save_state()
                await self.visualize_if_changed()

    async def get_task_info(self, task_id):
        async with self.lock:
            if self.graph.has_node(task_id):
                return self.graph.nodes[task_id].get('task_info')
        return None


