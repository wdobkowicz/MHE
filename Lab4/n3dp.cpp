#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <chrono>

#include "json.hpp"
// dot -Kfdp -n -Tpng example.dot  > example.png


class solution_t
{
public:
    virtual double goal() const = 0;
    virtual void next_solution() = 0;
};


struct n3dm_problem
{
    std::vector<int> X;
    std::vector<int> Y;
    std::vector<int> Z;
    int b{};
};

std::istream& operator>>(std::istream& os, n3dm_problem& problem)
{
    using nlohmann::json;
    json data;
    os >> data;
    data.at("problem").at("X").get_to<std::vector<int>>(problem.X);
    data.at("problem").at("Y").get_to<std::vector<int>>(problem.Y);
    data.at("problem").at("Z").get_to<std::vector<int>>(problem.Z);
    data.at("problem").at("b").get_to(problem.b);
    return os;
}

std::ostream& operator<<(std::ostream& os, const n3dm_problem& problem)
{
    os << "The numerical 3-dim matching problem: \n";

    std::array<char, 3> names = { 'X', 'Y', 'Z' };
    std::array<std::vector<int>, 3> to_print = { problem.X, problem.Y, problem.Z };
    for (std::size_t i = 0; i < names.size(); ++i) {
        os << "\t" << names.at(i) << " = {";
        auto vec = to_print.at(i);
        if (!vec.empty()) {
            for (auto it = vec.begin(); it != vec.end() - 1; it++) {
                os << *it << ", ";
            }
            os << vec.back();
        }
        os << "}\n";
    }
    os << "\tb = " << problem.b << "\n";
    return os;
}


// TODO it could be done without this struct and triples member of n3dm_solution_t
struct triple
{
    const int x, y, z;
    int get_sum() const
    {
        return x + y + z;
    }
};

std::ostream& operator<<(std::ostream& os, const triple& t)
{
    os << '(' << t.x << ", " << t.y << ", " << t.z << ") (sum=" << t.get_sum() << ")\n";
    return os;
}


class n3dm_solution_t : public solution_t
{
public:
    std::shared_ptr<n3dm_problem> problem;
    //std::vector<std::size_t> x_order;
    std::vector<std::size_t> y_order;
    std::vector<std::size_t> z_order;


    explicit n3dm_solution_t(const n3dm_problem& p)
    {
        problem = std::make_shared<n3dm_problem>(p);
        for (std::size_t i = 0; i < problem->X.size(); ++i) {
            //x_order.push_back(i);
            y_order.push_back(i);
            z_order.push_back(i);
        }
    }


    triple view_triple_at(std::size_t i) const
    {
        return { problem->X[i], problem->Y[y_order[i]], problem->Z[z_order[i]] };
    }

    auto as_triples() const
    {
        std::vector<triple> triples;
        for (std::size_t i = 0; i < problem->X.size(); ++i) {
            triples.push_back(view_triple_at(i));
        }
        return triples;
    }


    // let it be the count of triplets, for which the sum of its elements is not equal to expected b
    // if the goal == 0, then we have a solution
    double goal() const override
    {
        int sum = 0;
        for (std::size_t i = 0; i < y_order.size(); ++i) {
            sum += static_cast<int>(problem->b != view_triple_at(i).get_sum());
        }
        return sum;
    }

    void next_solution() override
    {
        bool reset = std::next_permutation(z_order.begin(), z_order.end());
        if (reset) {
            std::next_permutation(y_order.begin(), y_order.end());
        }
    }



    // void randomize_solution()
    // {
    //   using namespace std;
    //   static random_device rd;
    //   static mt19937 gen(rd());
    //   vector<int> cities;
    //   for (std::size_t i = 0; i < this->problem->size(); i++) {
    //     cities.push_back(i);// {0, 1, 2, 3 ...}
    //   }
    //   this->cities_to_see.clear();

    //   for (std::size_t i = 0; i < this->problem->size(); i++) {
    //     uniform_int_distribution<> dist(0, cities.size() - 1);
    //     int r = dist(gen);
    //     int randomly_selected = cities.at(r);
    //     cities.erase(cities.begin() + r);
    //     this->cities_to_see.push_back(randomly_selected);// {0, 1, 2, 3 ...}
    //   }
    // };

    // std::vector<tsp_solution_t> get_close_solutions() const
    // {
    //   std::vector<tsp_solution_t> ret;
    //   for (size_t i = 0; i < cities_to_see.size(); i++) {
    //     ret.push_back(*this);
    //   }

    //   for (size_t i = 0; i < cities_to_see.size(); i++) {
    //     std::swap(ret.at(i).cities_to_see.at(i),
    //       ret.at(i).cities_to_see.at((i + 1) % cities_to_see.size()));
    //   }
    //   return ret;
    // }
};

std::ostream& operator<<(std::ostream& os, const n3dm_solution_t& solution)
{
    os << "Triples:\n";
    for (std::size_t i = 0; i < solution.y_order.size(); ++i) {
        os << "\t" << solution.view_triple_at(i);
    }
    return os;
}

bool operator==(const n3dm_solution_t& a, const n3dm_solution_t& b)
{
    if (a.problem != b.problem)
        return false;
    if (a.y_order != b.y_order)
        return false;
    if (a.z_order != b.z_order)
        return false;
    return true;
}

// TODO te metaheurystyki (brute_force, hill_climbing etc.) powinny byæ zaimplementowane generycznie
n3dm_solution_t brute_force(const n3dm_problem& problem, bool verbose = true)
{
    using namespace std;
    n3dm_solution_t sol(problem);
    auto first_solution = sol;
    auto best_solution = sol;
    int iteration = 0;
    double current_best = best_solution.goal();

    if (verbose) std::cout << "Iter: " << iteration << ", current best goal: " << current_best << "\n";

    do {

        if (sol.goal() < current_best) {
            best_solution = sol;
            current_best = best_solution.goal();

            if (current_best == 0.0) {
                if (verbose) std::cout << "Iter: " << iteration << ", current best goal: " << current_best << " ...solution found.\n";
                break;
            }
            if (verbose) std::cout << "Iter: " << iteration << ", current best goal: " << current_best << "\n";
        }

        sol.next_solution();
        iteration++;

    } while (!(sol == first_solution));

    return best_solution;
}

// tsp_solution_t hill_climbing(const problem_t &problem)
// {
//   using namespace std;

//   tsp_solution_t sol;
//   sol.problem = make_shared<problem_t>(problem);

//   sol.randomize_solution();

//   auto first_solution = sol;
//   do {
//     // cout << "goal: " << sol.goal() << endl;
//     // sol.next_solution();
//     auto neighbours = sol.get_close_solutions();
//     auto best_neighbour = neighbours.back();
//     for (auto &neighbour : neighbours) {// najlepsze rozwiazanie z otoczenia
//       if (neighbour.goal() < best_neighbour.goal()) {
//         best_neighbour = neighbour;
//       }
//     }

//     if (sol.goal() > best_neighbour.goal()) {
//       sol = best_neighbour;
//     } else {/// warunek zakonczenia
//       return sol;
//     }
//   } while (true);
// }

// tsp_solution_t hill_climbing_nd(const problem_t &problem, unsigned int max_epochs)
// {
//   using namespace std;
//   using namespace std;
//   static random_device rd;
//   static mt19937 gen(rd());

//   tsp_solution_t sol;
//   sol.problem = make_shared<problem_t>(problem);

//   sol.randomize_solution();

//   auto first_solution = sol;
//   for (unsigned int i = 0; i < max_epochs; i++) {
//     auto neighbours = sol.get_close_solutions();
//     uniform_int_distribution<> dist(0, neighbours.size() - 1);
//     auto best_neighbour = neighbours.at(dist(gen));
//     if (sol.goal() >= best_neighbour.goal()) {
//       sol = best_neighbour;
//       cout << "[" << i << "]goal: " << sol.goal() << endl;
//     }
//   };
//   return sol;
// }

int main(int argc, char *argv[] )
{
    
    n3dm_problem problem;
    //std::cin >> problem;
    std::string fileName = argv[1];
    fileName.resize(fileName.size() - 5);
    std::cout << fileName + ".json\n\n";
    std::ifstream ifs;
    ifs.open(fileName + ".json");
    ifs >> problem;

    std::cout << problem;

    auto start = std::chrono::high_resolution_clock::now();

    n3dm_solution_t solution = brute_force(problem, true);

    constexpr double nano = 1e-9;
    constexpr int precision = 9;
    auto end = std::chrono::high_resolution_clock::now();
    double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    time_taken *= nano;

    std::cout << solution;
    std::cout << "Found by brute_force in: " << std::fixed << std::setprecision(precision) << time_taken << " sec.\n";

    std::ofstream fo;
    fo.open(fileName+".txt");
    fo << solution;

    return 0;
}