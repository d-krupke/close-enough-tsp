#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#include "gurobi_c++.h"

int main(int argc, char **argv) {
    doctest::Context context;
    context.applyCommandLine(argc, argv);

    int res = context.run(); // run doctest

    // important - query flags (and --exit) rely on the user doing this
    if (context.shouldExit()) {
        // propagate the result of the tests
        return res;
    }

    printf("%s\n", "Hello, World!");
}

int factorial(const int number) {
    return number < 1 ? 1 : number <= 1 ? number : factorial(number - 1) * number;
}

TEST_CASE("testing the factorial function") {
    CHECK(factorial(0) == 1);
    CHECK(factorial(1) == 1);
    CHECK(factorial(2) == 2);
    CHECK(factorial(3) == 6);
    CHECK(factorial(10) == 3628800);
}

void bla() {
    using namespace std;
  try {
    
    // Create an environment
    GRBEnv env = GRBEnv ( true );
    env.set ( "LogFile" , "mip1.log" );
    env.start ();
    // Create an empty model
    GRBModel model = GRBModel ( env );
    // Create variables
    GRBVar x = model.addVar (0.0 , 1.0 , 0.0 , GRB_BINARY , " x " );
    GRBVar y = model.addVar (0.0 , 1.0 , 0.0 , GRB_BINARY , " y " );
    GRBVar z = model.addVar (0.0 , 1.0 , 0.0 , GRB_BINARY , " z " );
    // Set objective : maximize x + y + 2 z
    model.setObjective ( x + y + 2 * z , GRB_MAXIMIZE );
    // Add constraint : x + 2 y + 3 z <= 4
    model.addConstr ( x + 2 * y + 3 * z <= 4 , " c0 " );
    // Add constraint : x + y >= 1
    model.addConstr ( x + y >= 1 , " c1 " );
    // Optimize model
    model.optimize ();
    cout << x.get ( GRB_StringAttr_VarName ) << " "
      << x.get ( GRB_DoubleAttr_X ) << endl ;
    cout << y.get ( GRB_StringAttr_VarName ) << " "
      << y.get ( GRB_DoubleAttr_X ) << endl ;
    cout << z.get ( GRB_StringAttr_VarName ) << " "
      << z.get ( GRB_DoubleAttr_X ) << endl ;
    cout << " Obj : " << model . get ( GRB_DoubleAttr_ObjVal ) << endl ;
  } catch ( GRBException e ) {
    cout << " Error code = " << e.getErrorCode () << endl ;
    cout << e.getMessage () << endl ;
  } catch (...) {
    cout << " Exception during optimization " << endl ;
  }
}
TEST_CASE("gurobi") {
    bla();
}