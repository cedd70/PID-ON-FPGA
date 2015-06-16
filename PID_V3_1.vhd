--#######################################
--#	Correcteur PID	V.3.1		#
--#######################################
library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	--use ieee.std_logic_signed.all;

entity PID_V3_1 is
	generic
	(
		------------ Coefficient ------------ 
		Kp 			: integer range 0 to 255:=1;
		Ki 			: integer range 0 to 255:=1;
		Kd			: integer range 0 to 255:=1
	);
	port 
	(
		------------ Input ------------ 
		clock,
		reset			: in std_logic;
		difference_in	   	: in std_logic_vector(31 downto 0 );
		------------ Output ------------ 
		consigne_end		: out std_logic:='0';
		commande_out		: out std_logic_vector(31 downto 0 )
	);

end entity;

architecture arch_PID of PID_V3_1 is

	------------ accuracy ------------ 
	constant	accuracy		: integer :=4;
	------------ clipping_threshold and Saturation Value ------------ 
	constant	clipping_threshold	: integer := 500;
	constant	value_saturation	: integer := 500;
	------------ output commande ------------ 
	signal   	commande_buffer		: integer range -2147483647 to 2147483647:=0;
	signal		commande		: integer range -2147483647 to 2147483647:=0;
	------------ difference between feedback and commande (PID) ------------ 
	signal 		difference		: integer range -2147483647 to 2147483647:=0;
	signal 		cpt_integrator		: integer range -2147483647 to 2147483647:=0;
	signal 		previous_difference	: integer range -2147483647 to 2147483647:=0;
	------------ Signal temp ------------ 
	signal 		temp_P			: integer range -2147483647 to 2147483647:=0;
	signal 		temp_I			: integer range -2147483647 to 2147483647:=0;
	signal 		temp_D			: integer range -2147483647 to 2147483647:=0;
	signal		i 			: integer range 0 to 5 :=0;
	begin

----------------------------------------------------------------------------------
	computation : process(clock,reset)
	begin
		if rising_edge(clock) then
			if reset = '0' then
				commande <= 0;
			else
			--loi de commande PID ( Pipe)
				for i in 0 to 1 loop 
					case i is 
						when 0 => 
							temp_P 		<= (difference * Kp);
						 	temp_I 		<= (cpt_integrator * Ki);
						 	temp_D		<= (difference - difference_previous) * Kd);
						when 1 => 
							commande_buffer 	<= temp_P + temp_I +temp_D;
						when others => null;
					end case;
				end loop;
			end if;
		end if;
	end process;
----------------------------------------------------------------------------------
	integrate : process(clock,reset)
	begin
		if reset = '0' then 
			cpt_integrator	  <=0;
		elseif rising_edge(clock) then
			if(difference < accuracy and difference > - accuracy) then
				cpt_integrator	 <= 0;
			else
				cpt_integrator	<=  cpt_integrator + difference;
			end if;
		end if;
	end process;
----------------------------------------------------------------------------------
	derivative: process(clock,reset)
	begin
		if reset = '0' then 
			previous_difference <= 0;
		elseif rising_edge(clock) then
			previous_difference <= difference;
		end if;
	end process;	
----------------------------------------------------------------------------------
	commande_process : process(clock,reset)
	variable tempo : integer range 0 to 30000000:=0;
	begin
		if (reset = '0') then 
			commande <= 0;
		elseif rising_edge(clock) then
			------------ stop Tuning ------------
			-- if ok for X time good position 
			if ((difference < accuracy) and (difference > - accuracy)) then 
				if tempo > 20000000 then
					commande		<= 0;
					consigne_end		<= '1';
				else
					tempo	:= tempo + 1 ;
				end if;
			------------	Clipping +	------------
			elsif (commande	> clipping_threshold)  then
				commande	<= value_saturation;
				consigne_end	<= '0';
				tempo		:=  0 ;
			------------	Clipping -	------------
			elsif (commande < - clipping_threshold) then
				commande	<= - value_saturation;
				consigne_end	<= '0';
				tempo		:=  0 ;			
			------------	Tuning	 ------------
			else
				commande	<= commande_buffer ;
				consigne_end	<= '0';
				tempo		:= 0 ;
			end if;	
		end if;	
	end process;
----------------------------------------------------------------------------------	
	difference			<= 	to_integer(signed(difference_in	));
	commande_out			<=  	std_logic_vector(to_signed(commande,commande'length));
end arch_PID;
